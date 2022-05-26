# Pacbot 2022

Welcome to the codebase for Tufts Robotics Club's 2022 Pacbot! The successfulness of this code is... debatable, but hey, at least we beat Princeton ;)

The codebase is split into 3 main parts: [actual_bot_code](actual_bot_code), which contains code to be run on the actual bot, [simulation](simulation), which contains the code for the pacman game simulation used in developing and testing the AI, and [misc_pi_stuff](misc_pi_stuff), which is an assortment of various test files that were later folded in to actual_bot_code. This README will focus on how to run both the simulation and actual bot code, as well as the general capabilities of each.

## Simulation

The game simulation was implemented by Harvard Undergraduate Robotics Club, and relevant documentation for how to set up and run the server, game engine, and visualization can be found [here](https://github.com/HarvardURC/Pacbot/tree/master/src/gameEngine#running-the-gameengine).

Our repo contains the keyboard input functionality used in those instructions, but also several other input module options for controlling the simulation. [keyboardInput2](simulation/keyboardInput2.py) is an adapted version of Harvard Undergraduate Robotics Club's keyboardInput module, which moves pacman moves only 1 grid square per key press instead of continuously moving in the current direction at each time step. [mouseInput](simulation/keyboardInput2.py) moves pacman based on the position of the cursor in the grid. [AIInput](simulation/AIInput.py) is the full, A*-based AI run on the robot in competition, and moves pacman autonomously based on the current game state at each time step. All of these can be run the same way as keyboardInput.py in order to control the simulation.

## Actual Bot Code

### General Setup

The code is divided into interconnected modules, all of which must be run at once (more on how to do that [here](README.md#how-to-run)). All of these modules pass messages to each other over a locally-running robomodules server on port 11295. Note that this is NOT the same as the robomodules server running the game; we get those messages from a different server. The [commsModule](actual_bot_code/commsModule.py) connects to both of these servers, and forwards all messages from the remote one to the local one, such that the other modules only have to connect to the local one. Yes we did shamelessly steal that code from Harvard <3

The other modules are as follows:

**AI Module**

Defined [here](actual_bot_code/aiModule.py). This module subscribes to the LIGHT_STATE messages from the game engine and uses that info to determine which direction pacman should go (W, A, S, or D). It then turns that into a PACMAN_DIRECTION message type and sends it to the server.

Note that, if you make any changes in to the AI code in simulation, you'll have to manually port them over to the versions of those files in actual_bot_code. As previously mentioned, these are pretty much the same (`smarterInput.py`, `Node.py`, and `HQ.py` are exactly the same), although I've changed the message sending a little and split the `tick` function from `AIInput.py` into two seperate ones in `actual_bot_code/aiModule.py`.

**Gyro Module**

Defined [here](actual_bot_code/gyroModule.py). This module reads raw angular velocity values from the gyro and uses them to update a stored angle approximation, representing how far the bot has turned since the last time the gyro has turned. It sends this angle approximation as a `GYRO_YAW` message whenever its value is updated, and also receives `GYRO_YAW` messages from other modules to signal when the stored angle should be zeroed.

**Motor Module**

Defined [here](actual_bot_code/motorModule.py). This module uses the `GYRO_YAW` and `PACMAN_DIRECTION` messages, as well as readings from the distance sensors, to drive the robot in the desired direction without running into walls.

**Reversible Motor Module**

Defined [here](actual_bot_code/reversibleMotorModule.py). This is a subclass of the regular motor module for use in bots with both front and back distance sensors, such that 180 degree turns are executed as simply reversals of the "forward" direction instead of actual turns.


**Test Module**

This one (defined [here](actual_bot_code/testCommandModule.py)) was thrown together for testing purposes, and drives the robot based on command line input (w/a/s/d/stop). Pretty much all it does is prompt for command line input, validate it, and then translate it into a PACMAN_DIRECTION message for use by the motor module.

### How to Run

While ssh-ed into the pi, after uploading the current version of actual_bot_code, cd into that directory and start running the local server with `./server.py`. It may tell you the file isn't runnable, which really just means it doesn't have the right permissions; you can change those with `chmod +x <path_to_file>`. Once you try running it again, it should work!

In another tab, (also sshed into the pi), do the same with `./aiModule.py` (or `./testCommandModule.py` for keyboard input), and, in a third tab, `./gyroModule.py`. You should not need to set any environment variables or anything for these.

You also need a motor module. In a bot with a fourth distance sensor on its rear, run (in an new tab, sshed into the pi) `./reversibleMotorModule.py`. Otherwise, run `./motorModule.py`. Similarly to the previously-described modules, neither of these should need any additional setup.

The comms module, on the other hand, does need environment variables. It also needs a remote game server to connect to. To set up a game server on your local machine, run [server.py](server.py) (the one in the base directory, NOT actual_bot_code) w/ the environment variable `BIND_ADDRESS` set to your public IP. Note that this will require the same setup/dependency installation steps as the simulation, which can be found [here](https://github.com/HarvardURC/Pacbot/tree/master/src/gameEngine#running-the-gameengine). Also run `gameEngine.py` if you want those game state messages. Then, while ssh-ed into the pi in the actual_bot_code directory, set the environment variable `BIND_ADDRESS` to that same IP and run `./commsModule.py`.

Now everything should be up and running! If not, uhh... blame Stinky. It's probably his fault.