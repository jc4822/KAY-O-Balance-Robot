import torch
from torchvision import transforms
from PIL import Image
import numpy as np
from model52 import DepthNet52
import os

SCALE = (72,96)

#input_dir = "/home/a1/Y2P_R/photos/in_rgb"
#output_dir = "/home/a1/Y2P_R/photos/out_png"
input_dir = "/home/a1/Y2P_R/resize_144/PairsSample"
output_dir = "/home/a1/Y2P_R/resize_144/PairsSample"
os.makedirs(output_dir, exist_ok=True) 

# 设备设置
device = torch.device("cpu")
model = DepthNet52().to(device)
model.load_state_dict(torch.load("Z_NYU_53_0.56.pth", map_location=device))
model.eval()

# 图像预处理
transform = transforms.Compose([
    transforms.Resize((144, 192)),  # (高度, 宽度)
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], 
                        std=[0.229, 0.224, 0.225])
])

for i in range(100,110):
    param_file = os.path.join(input_dir, f"{i:05}_K.npz")
        
    data = np.load(param_file)
    KRKiUV_Ts = [torch.from_numpy(data[f'KRKiUV_T_{72}x{96}'])]
    KT_Ts = [torch.from_numpy(data[f'KT_T_{72}x{96}'])]

    left_img = Image.open(os.path.join(input_dir, f"{i:05}_before.png")).convert('RGB')
    right_img = Image.open(os.path.join(input_dir, f"{i:05}_after.png")).convert('RGB')
        
    left_tensor = transform(left_img).unsqueeze(0).to(device)
    right_tensor = transform(right_img).unsqueeze(0).to(device)

    for j in range(1):
        KRKiUV_Ts[j] = KRKiUV_Ts[j].unsqueeze(0).to(device)
        KT_Ts[j] = KT_Ts[j].unsqueeze(0).to(device)

    refined_depth,_,_,_ = model(left_tensor, right_tensor, KRKiUV_Ts, KT_Ts, )

    depth_map = refined_depth.squeeze().cpu().detach().numpy()
    depth_map = (depth_map * 5000).clip(0, 65535).astype(np.uint16)
    Image.fromarray(depth_map).save(os.path.join(output_dir, f"{i:05}_out.png"))
        