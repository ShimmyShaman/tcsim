
# TODO / Notes

## Model-building workflow

> Generate image Dataset
> Run generateImageSets.py
> Run train_ssd command below:
>  - Move to pytorch-ssd directory
>  - python3 train_ssd.py --dataset-type=voc --data=/media/simpson/Backup/data/tennis_court/ --batch-size=10 --epochs=100 --debug-steps=400 --num-workers=4 --checkpoint-folder=/media/simpson/Backup/data/tennis_court/checkpoints
> Test:
>  - Move to pytorch-ssd directory
>  - python3 ./run_ssd_example.py mb1-ssd /media/simpson/Backup/data/tennis_court/checkpoints/mb1-ssd-Epoch-108-Loss-2.71-CL-0.82.pth /media/simpson/Backup/data/tennis_court/checkpoints/labels.txt /media/simpson/Backup/data/tennis_court/JPEGImages/ss_9469.jpg && eog ./run_ssd_example_output.jpg
> Trace Model for libtorch import:
>  - Move to tennis_court/py directory
>  - python3 trace_ssd_model.py /media/simpson/Backup/data/tennis_court/checkpoints/mb1-ssd-Epoch-203-Loss-2.71-CL-0.82.pth /media/simpson/Backup/data/tennis_court/checkpoints/labels.txt

# Video

- pick target according to current position AND ORIENTATION
- Pretty up the model a lil bit
- remove ball when run over
- add balls on occasion
- dont just transition markers (give a lower color markers to older detections (so it can be visually seen for longer than 0.37s)
- potentials to orange color
- make the primary marker a little more visually less abrasive


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