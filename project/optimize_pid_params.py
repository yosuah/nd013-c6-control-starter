import math
import subprocess
import time
import traceback

import pandas as pd

def run_with_params(p, throttle_op_mode, steer_op_mode):
    """
    Run the simulation with a given set of parameters
    
    p: 6 parameters passed to the simulation client
    throttle_op_mode: Throttle operation mode of the simulation. "constant_speed" for trying to maintain a constant speed 
                        without the controller, "constant_speed_reference" for asking the controller to maintain a constant speed or "normal"
                        for passing the output of the behavior planner and asking the controller to maintain that.
    steer_op_mode: Steer operation mode of the simulation. "straight" for fixed 0 steer output or "normal" for using the steer controller.
    """
    assert(len(p) == 6)
    # All parameters must be positive, short-circuit evaluation with the highest error if any of them are negative
    if any(map(lambda x: x < 0, p)):
        return math.inf

    # Enable one retry to overcome temporary glitches (e.g. the simulator did not start in time)
    retries = 0 
    while retries <= 1:
        if retries > 0: # things sometimes break randomly, allow one retry before failing
            print("..retrying")
        try:
            # Kill active Carla processes and wait for them to automatically respawn
            # to ensure that no dangling vehicles are left in the scene (probably this should be
            # added to simuatorAPI instead)
            # print("Killing Carla")
            p0=subprocess.run(["./kill_carla.sh"])
            time.sleep(4)

            # Read cumulated_data before running the program, so we can later check that there is 
            # exactly one new line in the file
            df = pd.read_csv('cumulated_data.txt')
            df_length_before = df.shape[0]

            # Start the simulation
            print("Starting the simulation client")
            p1=subprocess.Popen(["./pid_controller/pid_controller", 
                                 str(p[0]), str(p[1]), str(p[2]), str(p[3]), str(p[4]), str(p[5]), throttle_op_mode, steer_op_mode],
                                stdout=subprocess.PIPE)
            time.sleep(2)
            # print(f"Starting the simulator api")
            simulator_process = ["python3", "simulatorAPI.py"]
            if throttle_op_mode not in ["constant_speed", "constant_speed_reference"]:
                simulator_process.append("--add_obstacles")
            p2=subprocess.run(simulator_process, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

            print("Execution finished, getting results")

            # Read the results file again and get the error for the last execution
            df = pd.read_csv('cumulated_data.txt', index_col=None)
            if df.shape[0] == df_length_before + 1:
                row = df.iloc[-1]
                error = row['steer_mae'] + row['throttle_mae'] 
                return error
            else:
                print("Could not parse results, probably the simulation did not run correctly. Output from the programs:")
                print(p1.communicate())
                print(p2.communicate())
                retries += 1
        except Exception:
            print(f"Exception when running with arguments {p} {throttle_op_mode} {steer_op_mode}")
            traceback.print_exc()
            #print(p1.communicate())
            #print(p2.communicate())
            retries += 1

def print_params(p, dp, selected, method):
    result = "["
    for i in range(len(p)):
        add_dp = ""
        if i == selected:
            add_dp = f" {method} {dp[i]:.03f}"
        result += f"{', ' if i > 0 else ''}{p[i]:.03f}{add_dp}"
    result += "]"
    print(result)
    
            
def twiddle(p, dp, min_dp_threshold, sum_dp_threshold, throttle_op_mode, steer_op_mode):
    """
    Optimize a set of parameters using the twiddle algorithm
    
    p: Parameters to optimize
    dp: Initial magnitude of change to try when optimizing parameters (one for each parameter in p)
    min_dp_threshold: Smallest dp value for which to execute a simulation
    sum_dp_threshold: Minimum sum of values in dp when for starting a new iteration of the optimization
    throttle_op_mode: Throttle operation mode of the simulation. "constant_speed" for trying to maintain a constant speed 
                        without the controller, "constant_speed_reference" for asking the controller to maintain a constant speed or "normal"
                        for passing the output of the behavior planner and asking the controller to maintain that.
    steer_op_mode: Steer operation mode of the simulation. "straight" for fixed 0 steer output or "normal" for using the steer controller.
    """
    assert(len(p) == len(dp))
    dp_increase_ratio = 1.5
    dp_decrease_ratio = 0.5
    
    num_of_evaluations = 1
    best_error = run_with_params(p, throttle_op_mode, steer_op_mode)
    print(f"{best_error:.03f} Initial error, keeping params")
    
    while sum(dp) > sum_dp_threshold:
        for i in range(len(p)):
            # Skip execution if dp for this particular parameter is already too small (others might still be ok)
            if dp[i] <= min_dp_threshold:
                continue
            
            original_p = p[i]
            print_params(p, dp, i, '+')
            p[i] += dp[i]
            error = run_with_params(p, throttle_op_mode, steer_op_mode)
            num_of_evaluations += 1
            
            if error < best_error:
                best_error = error
                dp[i] *= dp_increase_ratio
                print(f"{error:.03f} Lower error, keeping new params")
            else:
                print(f"{error:.03f} Higher error, dropping params")
                p[i] = original_p
                print_params(p, dp, i, '-')
                p[i] -= dp[i]
                error = run_with_params(p, throttle_op_mode, steer_op_mode)
                num_of_evaluations += 1

                if error < best_error:
                    best_error = error
                    dp[i] *= dp_increase_ratio
                    print(f"{error:.03f} Lower error, keeping new params")
                else:
                    p[i] = original_p
                    dp[i] *= dp_decrease_ratio
                    print(f"{error:.03f} Higher error, dropping params")
    return p, best_error, num_of_evaluations
        
if __name__ == "__main__":
    with open('cumulated_data.txt', 'w') as f:
        f.write("steer_kp,steer_ki,steer_kd,throttle_kp,throttle_ki,throttle_kd,steer_mae,throttle_mae\n")
    
    # Set starting parameter set
    p = [0.125, 0, 0.0125, 1.425, 0.25, 0.525]
    
    num_of_evaluations = 0
    # First optimize only the throttle controller, without obstacles and for a constant reference speed
    
    # First find a working value for the throttle KP. Optimizing the other two parameters while the car can barely
    # start is just a waste of time. This can be skipped when already starting from a reasonable value.
    dp = [0.0, 0.0, 0.0, 0.1, 0.0, 0.0]
#     p, best_error, evals = twiddle(p, dp, 
#                                    min_dp_threshold=min(filter(lambda x: x > 0, dp)) / 10.0, 
#                                    sum_dp_threshold=sum(dp) / 10.0, 
#                                    throttle_op_mode="constant_speed_reference",
#                                    steer_op_mode="straight")
#     num_of_evaluations += evals
    
    # Find a value for KD and KI that stabilizes KP. Allow changes to KP again, because it is quite likely that values
    # that previously caused oscillation will now work fine.
    dp = [0.0, 0.0, 0.0, 0.1, 0.1, 0.1]
    p, best_error, evals = twiddle(p, dp, 
                                   min_dp_threshold=min(filter(lambda x: x > 0, dp)) / 10.0, 
                                   sum_dp_threshold=sum(dp) / 10.0, 
                                   throttle_op_mode="constant_speed_reference",
                                   steer_op_mode="straight")
    num_of_evaluations += evals
    
    # Optimize only the steer controller with constant speed and with obstacles (to force the reference trajectory
    # to be curved)
    dp = [0.1, 0.1, 0.1, 0.0, 0.0, 0.0]
    p, evals = twiddle(p, dp, 
                      min_dp_threshold=min(filter(lambda x: x > 0, dp)) / 10.0, 
                      sum_dp_threshold=sum(dp) / 10.0, 
                      throttle_op_mode="constant_speed",
                      steer_op_mode="normal")
    num_of_evaluations += evals
    
    print("")
    print(f"Total number of evaluations: {num_of_evaluations}")
    print(f"Best error: {best_error}")
    print(f"FINAL BEST PARAMS: {p}")