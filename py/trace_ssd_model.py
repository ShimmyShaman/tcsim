from vision.ssd.mobilenetv1_ssd import create_mobilenetv1_ssd, create_mobilenetv1_ssd_predictor
import torch

VOC_CLASSES = (  # always index 0
    'TennisBall')

model_path = "/home/simpson/proj/pytorch-ssd/models/mb1-ssd-Epoch-110+88-Loss-1.27-CL-1.03.pth"
label_path = "/home/simpson/proj/pytorch-ssd/models/labels.txt"

class_names = [name.strip() for name in open(label_path).readlines()]

net = create_mobilenetv1_ssd(len(class_names), is_test=True)
net.load(model_path)


#predictor = create_mobilenetv1_ssd_predictor(net, candidate_size=200)
net.eval()
print('Finished loading model!')
net = net.cuda()

img = torch.rand(1, 3, 300, 300).cuda()
resnet = torch.jit.trace(net, img)
resnet.save('ssd_voc.pt')
