import tensorflow as tf
from tensorflow import keras


from keras.layers import Lambda, Input, Dense
from keras.models import Model
from keras.datasets import mnist
from keras.losses import mse, binary_crossentropy
from keras.utils import plot_model
from keras import backend as K

import numpy as np
import matplotlib.pyplot as plt
import argparse
import os
import sys

from keras.preprocessing import image
from scipy import misc
from keras.layers import Input, Dense, Conv2D, MaxPooling2D, UpSampling2D, GlobalAveragePooling2D, Flatten, Reshape

# Build up training data:
# -Load a bunch of sample images.
# -For each image, iterate over all possible offsets to generate upsampled versions
#

# Need this 
physical_devices = tf.config.experimental.list_physical_devices('GPU')
if physical_devices:
    print(physical_devices)
    tf.config.experimental.set_memory_growth(physical_devices[0], True)


patch_size = 16
factor = 4
upsampled_size = patch_size * factor

def imresize(im, target_size, interp='lanczos'):
    output = np.zeros((target_size, target_size, 3))
    for i in range(0, 3):
        output[:,:,i] = misc.imresize(im[:,:,i].reshape(im.shape[0:2]), (target_size, target_size), interp=interp, mode='F')
    return output

def sample_image(image, skip=4):
    num_high = (image.shape[0] - upsampled_size) // skip + 1
    num_wide = (image.shape[1] - upsampled_size) // skip + 1
    large_images = np.zeros((num_high * num_wide, upsampled_size, upsampled_size, 3))
    small_images = np.zeros((num_high * num_wide, patch_size, patch_size, 3))
    print(num_high * num_wide)
    num = 0
    for i in range(0, image.shape[0] - upsampled_size, skip):
        for j in range(0, image.shape[1] - upsampled_size, skip):
            patch = image[i:(i+upsampled_size), j:(j+upsampled_size),:]
            small_images[num,:] = imresize(patch, patch_size, interp='lanczos')
            large_images[num,:] = patch #- imresize(small_images[num,:], upsampled_size, interp='lanczos')
            num = num + 1

    return large_images, small_images


def load_images(image_names):
    images = []
    large_images = []
    small_images = []
    for image_name in image_names:
        print(image_name)
        im = image.img_to_array(image.load_img(image_name)).astype('float32') / 255.
        l, s = sample_image(im)
        images.append(im)
        large_images.append(l)
        small_images.append(s)

    large_images = np.concatenate(large_images, axis=0)
    small_images = np.concatenate(small_images, axis=0)
    return images, large_images, small_images

def build_cnn(input_img):
    flat = False
    if flat:
        x = Flatten()(input_img)
        x = Dense(2048, activation='relu')(x)
        x = Dense(1024, activation='relu')(x)
        encoded = Dense(32, activation='relu')(x)
    else:
        x = Conv2D(64, (3, 3), activation='relu', padding='same')(input_img)
        x = MaxPooling2D((2, 2), padding='same')(x)
        x = Conv2D(16, (3, 3), activation='relu', padding='same')(input_img)
        encoded = MaxPooling2D((2,2), padding='same')(x)

    if flat:
        x = Dense(16 * 16 * 4 * 4 * 3, activation='sigmoid')(encoded)
        decoded = Reshape((16 * 4, 16 * 4, 3))(x)
    else:
        x = Conv2D(16, (1, 1), activation='relu', padding='same')(encoded)
        x = UpSampling2D((2, 2))(x)
        x = Conv2D(32, (3, 3), activation='relu', padding='same')(x)
        x = UpSampling2D((2, 2))(x)
        x = Conv2D(32, (3, 3), activation='relu', padding='same')(x)
        x = UpSampling2D((2, 2))(x)
        x = Conv2D(32, (1, 1), activation='relu', padding='same')(x)
        #    x = UpSampling2D((2, 2))(x)
        #    x = Conv2D(16, (1, 1), activation='relu', padding='same')(x)
        decoded = Conv2D(3, (1, 1), activation='sigmoid', padding='same')(x)
    return Model(input_img, decoded)

def upscale_image(img, model):
    num_blocks_high = img.shape[0] // patch_size
    num_blocks_wide = img.shape[1] // patch_size
    lowres = np.zeros((num_blocks_high * num_blocks_wide, patch_size, patch_size, 3))
    row = 0
    for i in range(0, patch_size * (img.shape[0] // patch_size), patch_size):
        for j in range(0, patch_size * (img.shape[1] // patch_size), patch_size):
            lowres[row,:,:,:] = img[i:(i+patch_size),j:(j+patch_size),:]
            row = row + 1
            print(row)
    highres = model.predict(lowres)
    output_img = np.zeros((num_blocks_high * patch_size * factor, num_blocks_wide * patch_size * factor, 3))
    row = 0
    for i in range(0, output_img.shape[0], patch_size * factor):
        for j in range(0, output_img.shape[1], patch_size * factor):
            output_img[i:(i+patch_size*factor),j:(j+patch_size*factor)] =  highres[row,:] # + imresize(lowres[row,:], upsampled_size)
            row = row + 1
    return output_img
    


input_img = Input(shape=(patch_size, patch_size, 3))
model = build_cnn(input_img)
model.summary()

image_names = sys.argv[1:]
images, large_images, small_images = load_images(image_names)


for i in range(0, 10):
    model.compile(optimizer='adam', loss='binary_crossentropy') #mean_squared_error')
    model.fit(small_images, large_images, epochs=10)

    p = plt.figure(figsize=(24, 24))
    ax = p.add_subplot(1, 2, 1)
    ax.imshow(upscale_image(images[0], model))
    ax = p.add_subplot(1, 2, 2)
    ax.imshow(upscale_image(images[1], model))
    plt.show()

