import os
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import numpy as np
from torchvision import transforms
from PIL import Image
from model53 import DepthNet53  # 假设您已有DepthNet52的实现
import math
import random

# 相机内参矩阵 (根据您的实际相机设置)
K = np.array([
    [155, 0, 95.5],
    [0, 155, 71.5],
    [0, 0, 1]
])

class DepthDataset(Dataset):
    def __init__(self, scene_dir, K, target_size=(144, 192)):
        self.scene_dir = scene_dir
        self.target_size = target_size
        self.K = K  # 相机内参矩阵 (3x3)
        self.samples = sorted([f for f in os.listdir(scene_dir) if f.endswith('_before.png')])
        self.scales = [(72,96), (36, 48), (18, 24), (9, 12)]  # 多尺度目标尺寸
        
        # 图像预处理
        self.rgb_transform = transforms.Compose([
            transforms.Resize(target_size),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], 
                                 std=[0.229, 0.224, 0.225])
        ])
        
        # 计算网格坐标用于相机参数计算
        H, W = target_size
        u = np.arange(0, W)
        v = np.arange(0, H)
        uu, vv = np.meshgrid(u, v)
        self.grid_points = np.stack([uu, vv, np.ones_like(uu)], axis=0).reshape(3, -1)  # (3, H*W)

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, idx):
        base_name = self.samples[idx].replace('_before.png', '')
        base_name = base_name[:5]
        param_file = os.path.join(self.scene_dir, f"{base_name}_K.npz")
        
        # 尝试加载预计算的参数
        if os.path.exists(param_file):
            data = np.load(param_file)
            KRKiUV_Ts = [torch.from_numpy(data[f'KRKiUV_T_{h}x{w}']) for h, w in self.scales]
            KT_Ts = [torch.from_numpy(data[f'KT_T_{h}x{w}']) for h, w in self.scales]
            loaded = True
        else:
            loaded = False
            KRKiUV_Ts, KT_Ts = [], []
        
        # 读取左右图像
        left_img = Image.open(os.path.join(self.scene_dir, f"{base_name}_before.png")).convert('RGB')
        right_img = Image.open(os.path.join(self.scene_dir, f"{base_name}_after.png")).convert('RGB')
        
        # 读取深度图 (使用before作为监督)
        depth = Image.open(os.path.join(self.scene_dir, f"{base_name}_depth_before.png"))
        
        # 读取变换矩阵
        transform_mat = np.loadtxt(os.path.join(self.scene_dir, f"{base_name}_position.txt"))
        
        # 应用预处理
        left_tensor = self.rgb_transform(left_img)
        right_tensor = self.rgb_transform(right_img)
        depth = torch.from_numpy(np.array(depth).astype(np.float32) / 5000.0)
        depth_tensor = depth.unsqueeze(0)
        
        # 如果没有加载参数，则计算并保存
        if not loaded:
            # 从4*4变换矩阵中提取旋转和平移
            R = transform_mat[:3, :3]
            t = transform_mat[:3, 3]
            
            # 计算相机参数
            invK = np.linalg.inv(self.K)
            KRKi = self.K @ R @ invK
            KT = self.K @ t.reshape(3, 1)
            
            # 计算全尺寸投影参数 (3 x H x W)
            KRKiUV_T = (KRKi @ self.grid_points).reshape(3, *self.target_size)
            KT_T = np.tile(KT, (1, self.grid_points.shape[1])).reshape(3, *self.target_size)
            
            # 转换为PyTorch张量
            KRKiUV_T = torch.from_numpy(KRKiUV_T).float()
            KT_T = torch.from_numpy(KT_T).float()
            
            # 多尺度下采样并保存
            save_data = {}
            for scale in self.scales:
                # 下采样投影参数
                krkiuv_resized = F.interpolate(
                    KRKiUV_T.unsqueeze(0), 
                    size=scale, 
                    mode='bilinear', 
                    align_corners=True
                ).squeeze(0)
                
                kt_resized = F.interpolate(
                    KT_T.unsqueeze(0), 
                    size=scale, 
                    mode='bilinear', 
                    align_corners=True
                ).squeeze(0)
                
                # 添加到返回列表
                KRKiUV_Ts.append(krkiuv_resized)
                KT_Ts.append(kt_resized)
                
                # 添加到保存数据
                h, w = scale
                save_data[f'KRKiUV_T_{h}x{w}'] = krkiuv_resized.numpy()
                save_data[f'KT_T_{h}x{w}'] = kt_resized.numpy()
            
            # 保存到文件
            np.savez(param_file, **save_data)
        
        # ===== 新增：生成模拟深度线传感器数据 =====
        H, W = self.target_size
        depth_line = torch.zeros(1, H, W)  # 创建全零张量
        
        # 随机选择扫描线的位置（高度）
        scan_line_height = random.randint(H//4, 3*H//4)  # 在图像中间区域随机选择
        
        # 随机选择扫描线宽度（5-10像素）
        scan_line_width = random.randint(5, 10)
        half_width = scan_line_width // 2
        
        # 获取真实深度图中的扫描线区域
        scan_region = depth_tensor[0, 
                                  max(0, scan_line_height-half_width):min(H, scan_line_height+half_width+1), 
                                  :].clone()
        
        # 添加噪声（±2cm）
        noise = torch.randn_like(scan_region) * 0.02  # 2cm噪声
        scan_region += noise
        
        # 对每列取平均，形成垂直的条形码效果
        col_means = scan_region.mean(dim=0, keepdim=True)
        
        # 将平均值复制到整个扫描线区域
        depth_line[0, 
                  max(0, scan_line_height-half_width):min(H, scan_line_height+half_width+1), 
                  :] = col_means.repeat(min(H, scan_line_height+half_width+1) - max(0, scan_line_height-half_width), 1)
        
        # 限制最大深度为2米（传感器探测范围）
        depth_line = torch.clamp(depth_line, 0, 2.0)
        
        return left_tensor, right_tensor, depth_tensor, KRKiUV_Ts, KT_Ts, depth_line

class MaskedL1Loss(nn.Module):
    def forward(self, pred, target):
        mask = (target > 0).detach()
        return torch.abs(pred[mask] - target[mask]).mean()

def train():
    scene_dir = '/home/a1/Y2P_R/resize_144/Pairs'  # 替换为实际路径
    save_dir = '/home/a1/Y2P_R/resize_144/Pairs'  # 替换为实际路径
    batch_size = 5
    epochs = 1
    lr = 1e-7
    os.makedirs(save_dir, exist_ok=True)
    
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Using device: {device}")
    
    # 初始化模型 - 现在需要接收额外的深度线输入
    model = DepthNet53(height=144, width=192).to(device)
    
    # 加载预训练权重（如果存在）
    model_path = "depth_model_53.pth"
    if os.path.exists(model_path):
        # 处理权重不匹配的情况
        pretrained_dict = torch.load(model_path, map_location=device)
        model_dict = model.state_dict()
        
        # 1. 过滤掉不匹配的权重
        pretrained_dict = {k: v for k, v in pretrained_dict.items() 
                          if k in model_dict and v.shape == model_dict[k].shape}
        
        # 2. 更新模型字典
        model_dict.update(pretrained_dict)
        
        # 3. 加载修改后的字典
        model.load_state_dict(model_dict)
        print(f"Loaded pretrained weights from {model_path} (with mismatched keys ignored)")
    
    # 数据集和数据加载器 - 现在返回6个元素
    dataset = DepthDataset(scene_dir, K)
    loader = DataLoader(dataset, batch_size=batch_size, shuffle=True, num_workers=4, pin_memory=True)

    # 损失函数和优化器
    criterion = MaskedL1Loss()
    optimizer = optim.AdamW(model.parameters(), lr=lr, weight_decay=1e-5)
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, 'min', factor=0.5, patience=2)

    best_loss = float('inf')
    for epoch in range(epochs):
        model.train()
        total_loss = 0.0
        batch_count = 0
        
        # 现在接收6个返回值，包括depth_line
        for left_imgs, right_imgs, depths, KRKiUV_Ts, KT_Ts, depth_lines in loader:
            # 移动数据到设备
            left_imgs = left_imgs.to(device)
            right_imgs = right_imgs.to(device)
            depths = depths.to(device)
            depth_lines = depth_lines.to(device)  # 新增的深度线输入
            
            # 前向传播 - 现在传入depth_lines
            optimizer.zero_grad()
            refined, out1, out2, out3 = model(left_imgs, right_imgs, KRKiUV_Ts, KT_Ts, depth_lines)
            
            # 计算多尺度损失
            loss_refined = criterion(refined, depths)
            loss_out1 = criterion(out1, depths)
            loss_out2 = criterion(out2, depths)
            loss_out3 = criterion(out3, depths)
            
            # 总损失（加权求和）
            total_batch_loss = loss_refined + 0.5 * (1.25*loss_out1 + loss_out2 + 0.75*loss_out3)
            
            # 反向传播
            total_batch_loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 0.5)
            optimizer.step()
            
            total_loss += total_batch_loss.item()
            batch_count += 1
            
            if batch_count % 50 == 0:
                print(f"Epoch {epoch+1}, Batch {batch_count}/{len(loader)}, Loss: {total_batch_loss.item():.4f}")
        
        epoch_loss = total_loss / len(loader)
        scheduler.step(epoch_loss)
        
        print(f"Epoch {epoch+1}/{epochs}, Loss: {epoch_loss:.4f}, LR: {optimizer.param_groups[0]['lr']:.2e}")
        
        # 保存最佳模型
        if epoch_loss < best_loss:
            best_loss = epoch_loss
            torch.save(model.state_dict(), model_path)
            print(f"Saved best model with loss: {best_loss:.4f}")
    
    # 使用最佳模型生成深度图
    model.load_state_dict(torch.load(model_path, map_location=device))
    model.eval()
    
    # 创建测试数据集（不进行数据增强）
    test_dataset = DepthDataset(scene_dir, K)
    
    with torch.no_grad():
        for j in range(100):
            left_img, right_img, depth, KRKiUV_T, KT_T, depth_line = test_dataset[j]
            
            # 添加批次维度
            left_img = left_img.unsqueeze(0).to(device)
            right_img = right_img.unsqueeze(0).to(device)
            depth_line = depth_line.unsqueeze(0).to(device)  # 新增
            
            for i in range(1):
                KRKiUV_T[i] = KRKiUV_T[i].unsqueeze(0).to(device)
                KT_T[i] = KT_T[i].unsqueeze(0).to(device)
            
            # 预测深度 - 现在传入depth_line
            refined_depth, _, _, _ = model(left_img, right_img, KRKiUV_T, KT_T, depth_line)
            
            # 后处理并保存
            depth_map = refined_depth.squeeze().cpu().numpy()
            depth_map = (depth_map * 5000).clip(0, 65535).astype(np.uint16)
            Image.fromarray(depth_map).save(os.path.join(save_dir, f"{j:05}_out.png"))

if __name__ == "__main__":
    train()