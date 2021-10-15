
# TODO / Notes

## Model-building workflow

> Generate image Dataset
> Run generateImageSets.py
> Run train_ssd command below:
>  - Move to pytorch-ssd directory
>  - python3 train_ssd.py --dataset-type=voc --data=/media/simpson/Backup/data/tennis_court/ --batch-size=10 --epochs=100 --debug-steps=400 --num-workers=4 --checkpoint-folder=/media/simpson/Backup/data/tennis_court/checkpoints
> Test:
>  - Move to pytorch-ssd directory
>  - python3 ./run_ssd_example.py mb1-ssd ./models/mb1-ssd-Epoch-110+88-Loss-1.27-CL-1.03.pth ./models/labels.txt /home/simpson/data/tennis_court/JPEGImages/ss_111.jpg && eog ./run_ssd_example_output.jpg
> Trace Model for libtorch import:
>  - Move to tennis_court/py directory
>  - /media/simpson/Backup/data/tennis_court/checkpoints/labels.txt


## TODO

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