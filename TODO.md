
* control pipeline
  - transfer to deep model
  - model analysis
  - returns actions
* motion model
  - lock Z
  - control through left & right motor in 2 directions each
* git important data -- remove it from .gitignore maybe
* increase ground texture detail
* camera angle
* tennis ball

Algorithm
- Ball spotting algorithm - like k-means clustering algorithm
- obstruction grid
- movement vocabulary and updating pathfinder

Problems to solve
- Pathing: Develop a way to generate an arbitrary path and follow it
  - Give a system a number of 2D locations. Have the system generate a path and have it followed
- Ball Spotting: Develop an algorithm that determines spots that balls are likely at
- Obstruction:
  - Find a means to detect it
  - Design a pathing system that avoids obstacles & runs over (past) ball spots
- Localization: Tennis Lines? Figure out how to stay on/near the court