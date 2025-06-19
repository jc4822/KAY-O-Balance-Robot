import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
import math
from imported import LiteMono
import time

class DepthHead(nn.Module):
    def __init__(self, in_channels, out_channels=1, target_size=(144, 192)):
        super().__init__()
        self.target_size = target_size
        self.conv1 = nn.Conv2d(in_channels, 32, kernel_size=3, padding=1)
        self.conv2 = nn.Conv2d(32, 16, kernel_size=3, padding=1)
        self.conv3 = nn.Conv2d(16, out_channels, kernel_size=1)
        self.relu = nn.ReLU(inplace=True)
        
    def forward(self, x):
        x = self.relu(self.conv1(x))
        x = self.relu(self.conv2(x))
        x = self.conv3(x)
        
        if x.shape[2:] != self.target_size:
            x = F.interpolate(
                x, 
                size=self.target_size, 
                mode='bilinear', 
                align_corners=True
            )
        return x

class DepthNet53(nn.Module):
    def __init__(self, model_type='lite-mono', height=144, width=192):
        super().__init__()

        self.depth_bins = 48
        self.idepth_base = 1.0 / 50.0
        self.idepth_step = (1.0 / 0.5 - 1.0 / 50.0) / (self.depth_bins - 1)

        self.feature_extractor = nn.Sequential(
            nn.Conv2d(3, 16, kernel_size=3, stride=1, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(16, 16, kernel_size=3, stride=1, padding=1),
            nn.ReLU(inplace=True)
        )

        self.cost_volume_compress = nn.Sequential(
            nn.Conv2d(self.depth_bins, 16, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.Upsample(scale_factor=2, mode='bilinear', align_corners=True)
        )

        self.encoder = LiteMono(
            in_chans=20,
            model=model_type,
            height=height,
            width=width,
        )
        
        enc_channels = self.encoder.num_ch_enc
        
        self.depth_head_full = DepthHead(enc_channels[0])
        self.depth_head_half = DepthHead(enc_channels[1])
        self.depth_head_quarter = DepthHead(enc_channels[2])

        self.fusion_conv = nn.Sequential(
            nn.Conv2d(3, 16, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(16, 1, kernel_size=1)
        )
        
    def get_volume(self, left_feat, right_feat, KRKiUV_T, KT_T): 
        B, C, H, W = left_feat.shape

        #KRKiUV_T= F.interpolate(KRKiUV_T, size=(72,96), mode='bilinear', align_corners=True)
        #KT_T = F.interpolate(KT_T, size=(72,96), mode='bilinear', align_corners=True)

        depth_indices = torch.arange(0, self.depth_bins, device=left_feat.device)
        depths = 1.0 / (self.idepth_base + depth_indices * self.idepth_step)
        depths = depths.view(1, -1, 1, 1, 1)  # [1, D, 1, 1, 1]

        KRKiUV_T = KRKiUV_T.unsqueeze(1)  # [B, 1, 3, H, W]
        KT_T = KT_T.unsqueeze(1)

        transformed = KRKiUV_T * depths + KT_T

        normalize_base = torch.tensor([W / 2.0, H / 2.0], 
                                    device=left_feat.device).view(1, 1, 2, 1, 1)
        
        demon = transformed[:, :, 2:3, :, :]  # [B, D, 1, H, W]
        warp_uv = transformed[:, :, 0:2, :, :] / (demon + 1e-6)
        warp_uv = (warp_uv - normalize_base) / normalize_base

        warp_uv = warp_uv.permute(0, 1, 3, 4, 2).reshape(B * self.depth_bins, H, W, 2)

        right_feat_sampled = right_feat.unsqueeze(1).repeat(1, self.depth_bins, 1, 1, 1)
        right_feat_sampled = right_feat_sampled.reshape(B * self.depth_bins, C, H, W)

        warped = F.grid_sample(
            right_feat_sampled, 
            warp_uv, 
            align_corners=True, 
            padding_mode='border'
        )

        warped = warped.view(B, self.depth_bins, C, H, W)

        left_feat_expanded = left_feat.unsqueeze(1)  # [B, 1, C, H, W]
        cost_volume = torch.sum(torch.abs(warped - left_feat_expanded), dim=2)  # [B, D, H, W]
        
        return cost_volume
        
    def forward(self, left_img, right_img, KRKiUV_T, KT_T, depth_line):
        #t1 = time.time()

        half_size = (left_img.shape[2] // 2, left_img.shape[3] // 2)
        left_half = F.interpolate(left_img, size=half_size, mode='bilinear', align_corners=True)
        right_half = F.interpolate(right_img, size=half_size, mode='bilinear', align_corners=True)

        left_feat = self.feature_extractor(left_half)
        right_feat = self.feature_extractor(right_half)

        cost_volume = self.get_volume(left_feat, right_feat, KRKiUV_T[0], KT_T[0])

        cost_volume = self.cost_volume_compress(cost_volume)

        encoder_input = torch.cat([left_img, cost_volume, depth_line], dim=1)

        features = self.encoder(encoder_input)
        
        depth_full = self.depth_head_full(features[0])
        depth_half = self.depth_head_half(features[1])
        depth_quarter = self.depth_head_quarter(features[2])

        fused = torch.cat([depth_full, depth_half, depth_quarter], dim=1)
        fused_depth = self.fusion_conv(fused)
        
        #t2 = time.time()
        #print(t2 - t1)

        return fused_depth, depth_full, depth_half, depth_quarter