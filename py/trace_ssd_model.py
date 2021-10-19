from torch._C import device
from vision.ssd.mobilenetv1_ssd import create_mobilenetv1_ssd, create_mobilenetv1_ssd_predictor
import torch
import sys

if len(sys.argv) < 2:
    print('Usage: python run_ssd_example.py <image path> <label_path>')
    sys.exit(0)
try:
    model_path = sys.argv[1]
    label_path = sys.argv[2]
except:
    print('Usage: python run_ssd_example.py <image path> <label_path>')
    sys.exit(0)


VOC_CLASSES = (  # always index 0
    'TennisBall')

class_names = [name.strip() for name in open(label_path).readlines()]

net = create_mobilenetv1_ssd(len(class_names), is_test=True, device="cpu")
net.load(model_path)


# predictor = create_mobilenetv1_ssd_predictor(net, candidate_size=200)
# net = net.to(device='cpu')
# net.eval()
print('Finished loading model!')


img = torch.rand(1, 3, 300, 300).to(device='cpu') #.cuda()
# res = net.forward(img);
# torch.set_printoptions(threshold=10_000)
# print(len(res))
# print(res[0].size())
# print(res[1].size())
resnet = torch.jit.trace(net, img)
resnet.save('ssd_voc.pt')
print('Model Traced & Saved!')
