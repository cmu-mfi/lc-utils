#!/usr/bin/env python3

# Useful for analyzing how power fluctuates over time.
# Used together with lcw_save_imgs.py

from pathlib import Path
import cv2
import numpy as np
import matplotlib.pyplot as plt

threshold = 50

def calculate_mean_intensity(image_path):
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    indices = np.where(image > threshold)
    return np.mean(image[indices])

def calculate_max_intensity(image_path):
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    indices = np.where(image > threshold)
    return np.max(image[indices])

def calculate_px_intensity(image_path):
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    return image[350, 420]

def plot_mean_intensity(folder_path):
    folder = Path(folder_path)
    ir_image_paths = sorted(folder.glob('ir*.png'))
    raw_image_paths = sorted(folder.glob('raw*.png'))

    mean_raw_intensities = []
    mean_ir_intensities = []
    max_raw_intensities = []
    max_ir_intensities = []
    px_raw_intensities = []
    px_ir_intensities = []

    for ir_image_path, raw_image_path in zip(ir_image_paths, raw_image_paths):
        mean_raw_intensities.append(calculate_mean_intensity(raw_image_path))
        max_raw_intensities.append(calculate_max_intensity(raw_image_path))
        px_raw_intensities.append(calculate_px_intensity(raw_image_path))

        mean_ir_intensities.append(calculate_mean_intensity(ir_image_path))
        max_ir_intensities.append(calculate_max_intensity(ir_image_path))
        px_ir_intensities.append(calculate_px_intensity(ir_image_path))

    # Plot the mean and max intensities of raw and IR images in a subfigure
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1)
    ax1.plot(mean_raw_intensities, label='Mean Raw Intensity')
    ax1.plot(mean_ir_intensities, label='Mean IR Intensity')
    ax1.set_title('Mean Intensities')
    ax1.legend()
    ax2.plot(max_raw_intensities, label='Max Raw Intensity')
    ax2.plot(max_ir_intensities, label='Max IR Intensity')
    ax2.set_title('Max Intensities')
    ax2.legend()
    ax3.plot(px_raw_intensities, label='Px Raw Intensity')
    ax3.plot(px_ir_intensities, label='Px IR Intensity')
    ax3.set_xlabel('Seconds')
    ax3.set_title('Fixed px. Intensities')
    ax3.legend()
    plt.show()

folder_path = './images_july6_1700/'  # Replace with the actual folder path containing the images
plot_mean_intensity(folder_path)
