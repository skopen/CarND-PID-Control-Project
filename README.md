# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---


## Reflection on the PID Controller Program
Here is the impact of P, I, D component of the PID algorithm on my program.

#### P (Proportional) Component
The P component had the biggest impact on the magnitude of the control output, because this is the component that causes a proportional change in the magnitude of the control output. This also caused the widest swings, if I choose a higher value, especially in the absence of other conponents, such as the D component.

#### I (Integral) Component
This term really helps compensate for any biases or drifts in the system. Since there is presumably not much drift in our system, we notice that the value of the I component is very close to zero. For our setup, this was the least influential of the components.

#### D (Derivative) Component
This component helps reduce the control output magnitude as we start seeing the CTE (cross track error) reduce with time very fast. When the CTE error reduces, the Derivative of CTE with respect to time, will be negative, hence we are effectively damping out the control output. This helps reduce the wide swings we would otherwise see, if we were just operating on the P component.

## Hyperparameter Choosing
This was probably the more challenging part of the program. Here are the rough steps I followed:

#### Manual Probing/Tuning
I first manually probed various values that can keep the car on the road and reduce the swings. First thing I noted is that, I cannot have all of the P-I-D components 0, as the car will just not be able to stay on the road. So I started changing various values (starting with lower values). I tried the followng values manually:

* pid.Init(0.4, 0.004, 4); --> very unstable ride, swerves wildly to left and right
* pid.Init(0.4, 0.04, 4); --> less swerving ride
* pid.Init(0.4, 0.4, 4); --> Very bad ride
* pid.Init(0.4, 0.4, 0.4); --> Very bad ride
* pid.Init(0.4, 0.004, 2); --> Somewhat stable
* pid.Init(0.1, 0.004, 1);  --> More stable
* pid.Init(0.07, 0.004, 1); --> Somewhat stable
* pid.Init(0.12, 0.004, 1); --> Best manually tuned ride

#### Tuning using Twiddle algorithm
I implemented the Twiddle algorithm as explained in the lectures. The main challenge using Twiddle was that, I had to start with more reasonable starting values for the hyperparameters, as otherwise the car would not stay on the road long enough for the tuning to proceed using Twiddle. Another challenge was how to inject Twiddle into the given code because the driver for the code were continuous events from the Simulator. I did this by chunking the continuous input data into batches over which I would iterate on parameters (from batch to batch). I had to experiment with different batch sizes (MAX_ITER). I had to make two modes for the program to operate in:

* Tuning mode: When "tune" command line parameter passed, the program first tunes and then continues with the tuned parameters.
* Running mode: When no command line parameter is passed, the program runs with pre-tuned parameters set in code.

Overall, here are the results I decide to continue with:

* Starting parameters (PID): 0.07, 0.003, 0.6
* Final Tuned Parameters (PID): 0.184845, 0.00911653, 1.4098

The videos for my Twiddle based tuning exercise are recorded [here](https://youtu.be/zQFBAXSPSzc).
The video of my car go around one lap on the given track is recorded [here](https://youtu.be/cU468pyPBMk).

## Dependencies
* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

