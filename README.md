# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

## 0. Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## 1. PID Controller

### 1.1 Proportional Gain
...

### 1.2 Derivative Gain
...

### 1.3 Integral Gain
...

## 2. Steering Controller
The simulator provides the cross track error (CTE), speed, and steering angle at each time step which can be used to calculate the corrective actions required. Initially, conservative parameters were set for the steering PID controller to rely more on the proportional parameters rather than the integral and derivative components resulting in an oscillating driving behaviour and a slow speed (20% throttle). Since the steering angle is [-1,1] all steering values from the controller were mapped to match this range. The speed was set to 20% (0.20) for this portion of the tuning and the PID gains were determined heuristically to be Kp=0.05, Ki=0.005, and Kd=1.5 resulting in a wobbly drive around the track. Lastly, just like in modern vehicles, as the speed increases the steering wheel "stiffens" to reduce the output angle. We mimick this by reducing the steering out down to 25% at max speed and 100% at the minimum speed.

## 3. Throttle Controller
In the real world drivers tend to drive faster along straight sections of road when their CTE is lower and it is easier to stay in the center of the lane. Likewise, when entering into a turn and the CTE increases as it is more difficult to maintain lane center we drive slower as a result. This behaviour can be simulated by passing the throttle controller the CTE and have it respond inversely to how the steering controller works; as system error decreases the result (throttle) increases. 

## 4. Parameter Optimization
The car now drives around the track but as mentioned in the course lectures, all this swaying would make the passengers seasick after a little while. Rather than heuristically tuning these controllers we can implement a function called Twiddle that dynamically adjusts the PID parameters based on system response and overall error.

<p align="center">
 <img src="./res/tuning.gif">
</p>

### 4.1 Twiddle Function
This is a useful method for automating the tuning process for a PID controller. It works by deviating the gain parameters from their current value by a preset step size and then comparing the overall error after each state. The first step increases the gain by one step, but if that results in higher overall system error then it subtracts that tep and then steps once more in the negative direction. From these state changes, the gain step sizes are adjusted accordingly (+/-5%). As the controller converges on the optimal tuning, the step sizes slowly decrease as a result.

	def twiddle(tol=0.2): 
	    p = [0, 0, 0]
	    dp = [1, 1, 1]
	    robot = make_robot()
	    x_trajectory, y_trajectory, best_err = run(robot, p)

	    it = 0
	    while sum(dp) > tol:
	        print("Iteration {}, best error = {}".format(it, best_err))
	        for i in range(len(p)):
	            p[i] += dp[i]
	            robot = make_robot()
	            x_trajectory, y_trajectory, err = run(robot, p)

	            if err < best_err:
	                best_err = err
	                dp[i] *= 1.05
	            else:
	                p[i] -= 2 * dp[i]
	                robot = make_robot()
	                x_trajectory, y_trajectory, err = run(robot, p)

	                if err < best_err:
	                    best_err = err
	                    dp[i] *= 1.05
	                else:
	                    p[i] += dp[i]
	                    dp[i] *= 0.95
	        it += 1
	    return p

Since Twiddle is intended to work on a full data set where the data sets are identical, the challange for this project was to implement it on a running system that is consistently receiving new and unique data. To assist the function with this we add states and iteration loops to set "periods" to record error from and use these periods to adjust the parameters. The drawback here is that the controller will "untune" itself when on corners and therefore will take longer to converge on the optimal settings. 

### 4.2 Tuning the Controller
The minimum car speed is set to 10 mph where the Twiddle function does not activate below 15 mph to ensure that the car will speed up as it converges on it's optimal tuning and slow down when becoming unstable. This essentially turns the optimization OFF when on the corners on the track and activates on the straight away. Letting the program run for a little while will allow the parameters to converge on their ideal values and the car will speed up.

## 5. Running the Simulator

<p align="center">
 <img src="./res/optimized.mp4">
</p>


