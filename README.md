# Pacbot-2022

I've decided to kinda sorta write a readme about the bot code because it seems bad that I'm the only one who knows how to run it. I will be... pleasantly surprised if either of you actually reads this but hey. A girl can dream.

All of the bot code is in the [actual_bot_code](actuual_bot_code) directory. You may see some repeated files/directories in there that also exist elsewhere in this repo. Could I refactor that? Possibly. Am I going to? No. Also this way it's really easy to just upload the whole directory w/ everything you need to the pi at once.

## General Setup

The code is divided into interconnected modules, all of which must be run at once (more on how to do that later). All of these modules pass messages to each other over a locally-running robomodules server on port 11295. Note that this is NOT the same as the robomodules server running the game; we get those messages from a different server. The [commsModule](actual_bot_code/commsModule.py) connects to both of these servers, and forwards all messages from the remote one to the local one, such that the other modules only have to connect to the local one. Yes I did shamelessly steal that code from Harvard <3

The other modules are as follows:

### AI Module

Defined [here](actual_bot_code/aiModule.py). This is essentially the same as AI Input; it subscribes to the LIGHT_STATE messages from the game engine and uses that info to determine which direction pacman should go (W, A, S, or D). It then turns that into a PACMAN_DIRECTION message type and sends it to the server.

Note that, if you make any changes in to the AI code in simulation, you'll have to manually port them over to the versions of those files in actual_bot_code. As previously mentioned, these are pretty much the same (`smarterInput.py`, `Node.py`, and `HQ.py` are exactly the same), although I've changed the message sending a little and split the `tick` function from `AIInput.py` into two seperate ones in `actual_bot_code/aiModule.py`.

### Motor/Sensor Module

Defined [here](actual_bot_code/motorModule.py). Currently this is all one module, though it's kind of a mess so I've been considering splitting it up into multiple ones (gyro module, distance sensors module, and motor module) if it doesn't increase the latency too much. Most of this is a modified, object-oriented version of Ryan's python scripts, so hopefully most of it will look familiar. This module subscribes to the AI module's PACMAN_DIRECTION messages, compares those desired directions to the actual state of the bot, and uses the sensors and motors to drive the bot accordingly.

### Test Module
This one (defined [here](actual_bot_code/testCommandModule.py)) I threw together for testing purposes, and drives the robot based on command line input (w/a/s/d/stop). Pretty much all it does is prompt for command line input, validate it, and then translate it into a PACMAN_DIRECTION message for use by the motor module.

## How to Run

While ssh-ed into the pi, after uploading the current version of actual_bot_code, cd into that directory and start running the local server with `./server.py`. It may tell you the file isn't runnable, which really just means it doesn't have the right permissions; you can change those with `chmod +x <path_to_file>`. Once you try running it again, it should work!

In another tab, (also sshed into the pi), do the same with `./aiModule.py` (or `./testCommandModule.py` for keyboard input), and, in a third tab, `./receiverModule.py`. You should not need to set any environment variables or anything for these.

The comms module, on the other hand, does need environment variables. It also needs a game engine to connect to. On your (meaning Ryan's, presumably) local machine, run [server.py](server.py) (the one in the base directory, NOT actual_bot_code) w/ the environment variable `BIND_ADDRESS` set to your public IP. Also run `gameEngine.py` if you want those game state messages. Then, while ssh-ed into the pi in the actual_bot_code directory, set the environment variable `BIND_ADDRESS` to that same IP and run `./commsModule.py`.

Now everything should be up and running! If not, uh... that probably means I broke something so feel free to yell at me over the phone.
