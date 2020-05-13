# pywalker - Making a 2D bot walk

The goal of this project is to create simple 2D simulation where the bot learns to walk on its own. The purpose is to learn using NEAT algorithm to create an AI. While OpenAI's gym already provides a environment to simulate such a scenario, for learning purpose I am trying to make my own implmentation of this environment using Box2D.

Initially it will be a plain ground, where the bot will have two legs with a uppper box body. The legs will be broken into two boxes, connected with an upper and lower joint. The target will be to have a bot that will control the motor speed and torque of these joints to make it move.
