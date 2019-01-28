# MuscledAgents
OpenAI Gym MuJoCo models rigged with muscles and environments which
incorporate [PyMuscle](http://github.com/iandanforth/pymuscle) fatigable muscle models.

<img src="https://user-images.githubusercontent.com/446062/51863364-a7db2680-22f5-11e9-97e9-582feccf44c6.png" width="50%">

## Prerequisites

 - MuJoCo 1.5
 - mujoco-py

## Setup

```
python setup.py develop
```

## Getting Started

The only functional environment is currently `ant`. Please run that as follows

```
cd agents
python ant.py
```

If things go well you'll see the standard `ant` model now rigged with tendons
and moving in a simple dance-like pattern.

## Usage

To explore these environments and models further you will need to understand
how input values get translated into final simulated movements.

### Control Signals

The action space for an ant is continuous control over 16 muscles. For
each of four legs there are four muscles. One leg extensor, one leg flexor,
and two hip muscles which move the leg left and right (or forward and back
depending on your perspective.)

### Gym Environment

The `step` method takes an array of 16 values which represent the input to
the fatigable muscle model for each muscle. Inputs should be in the range [0-1].

### PyMuscle Fatigue

After use muscles produce less force for the same level of input. So if you
were to send an input signal which recruited all motor units in a muscle
constantly for several seconds the output the model will return will rapidly
decrease. A period of light or no use is required for the muscle to recover.

### MuJoCo Model

Each tendon actuator is control limited to the range [-1.0, 0.0]. When a
General actuator is tied to a Tendon in MuJoCo negative values are the
equivalent of contractions. Muscles cannot produce force in extension so no
positive non-zero values are allowed.

Actuators have a `gainprm` which scales this input value. This is tuned to
a value of 100 to work with the mass of the ant and the resistances of opposing
tendons.

## Altering the Models

Due to frustration editing XML files the `ant` model is generated by a python
script that uses the [mjcf](http://github.com/iandanforth/mjcf) library which 
I wrote to wrap MuJoCo xml elements in python classes. Note: This may
be replaced with the mjcf library from [dm_control](https://github.com/deepmind/dm_control/tree/master/dm_control/mjcf) in the future.

You can view and modify that script in `scripts/gen_ant.py`.
