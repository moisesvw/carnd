
import csv
import cv2
import numpy as np

import tensorflow as tf
from keras.models import Sequential
from keras.layers.core import Dense, Activation, Flatten, Dropout, Lambda
from keras.layers.convolutional import Convolution2D, Cropping2D
from keras.layers.pooling import MaxPooling2D
from keras import optimizers

from sklearn.model_selection import train_test_split
import sklearn
import random
import pandas as pd

def generator(samples, batch_size=128, add_flip_image=False):
    """
    Generates random batches of data base on image path samples
    It could add flipped images if incoming samples contains
    Left and Rigth cameras images
    """

    num_samples = len(samples)
    while True:
        random.shuffle(samples)
        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset:offset+batch_size]
            
            images = []
            angles = []
            for batch_sample in batch_samples:
                data_dir = batch_sample[3].strip()
                image_path = data_dir + batch_sample[0].strip()
                image = cv2.imread(image_path)
                image = cv2.cvtColor(image, cv2.COLOR_BGR2YUV)
                steering = float(batch_sample[2])

                if batch_sample[1] == 'L':
                    steering+= 0.22
                elif batch_sample[1] == 'R':
                    steering-= 0.22
                    
                images.append(image)
                angles.append(steering)

                #Augmentation flip images
                if add_flip_image:
                    images.append(cv2.flip(image, 1))
                    angles.append(steering * -1.0)
                
            X_train = np.array(images)
            y_train = np.array(angles)
            
            yield sklearn.utils.shuffle(X_train, y_train)

def normalize_pixels(x):
    """
    Normalize image data
    """
    return x / 255.0 - 0.5
            

def train_genarator_model(model, train_generator, steps_per_epoch,
                          validation_generator, validation_steps, epochs=1):
    """
    Train a model with adam optimizer and mse
    """

    optimizer = optimizers.Adam(lr=0.0006)
    
    model.compile(loss='mse', optimizer=optimizer)
    model.fit_generator(train_generator,
                        steps_per_epoch=steps_per_epoch,
                        validation_data=validation_generator,
                        validation_steps=validation_steps,
                        epochs=epochs)
    return model

def get_data(all_cameras=False, data_source={"./track2d/driving_log.csv": "./track2d/"}):
    """
    generates image data base on its data recorded from cameras in simulator
    """
    samples = []
    CENTER, LEFT, RIGTH, STEERING = 0, 1, 2, 3
    for path, directory in data_source.items():
        with open(path) as csvfile:
            reader = csv.reader(csvfile)
            next(reader, None)
            for line in reader:
                if all_cameras:
                    samples.append([line[CENTER], 'C',line[STEERING], directory ])
                    samples.append([line[LEFT],   'L',line[STEERING], directory ])
                    samples.append([line[RIGTH],  'R',line[STEERING], directory ])
                else:
                    samples.append([line[CENTER], 'C',line[STEERING], directory ])
            
    return samples

data_source = {
    "./train2/driving_log.csv": "./train2/",
    "./track2d/driving_log.csv": "./track2d/",
}

all_images_data = get_data(all_cameras=True, data_source=data_source)
train_samples, validation_samples = train_test_split(all_images_data, test_size=0.2)
validation_samples, test_samples  = train_test_split(validation_samples, test_size=0.5)

generator_batch = 64
add_flip_image = True

train_generator = generator(train_samples, batch_size=generator_batch, add_flip_image=add_flip_image)
validation_generator = generator(validation_samples, batch_size=generator_batch)
test_generator = generator(test_samples, batch_size=generator_batch)

train_steps      =  ((len(train_samples)*2)/generator_batch) if add_flip_image else len(train_samples)/generator_batch
validation_steps = len(validation_samples)/generator_batch



def nvida_model():
    """
    NVIDIA Model based on paper
    https://arxiv.org/abs/1604.07316
    """
    model = Sequential()
    model.add(Cropping2D(cropping=((65, 25), (9,9)), input_shape=(160,320,3)))
    model.add(Lambda(normalize_pixels))
    model.add(Convolution2D(24, (5, 5), strides=(2, 2), padding='valid', activation='relu'))
    model.add(Convolution2D(36, (5, 5), strides=(2, 2), padding='valid', activation='relu'))
    model.add(Convolution2D(48, (5, 5), strides=(2, 2), padding='valid', activation='relu'))
    model.add(Convolution2D(64, (3, 3), strides=(1, 1), padding='valid', activation='relu'))
    model.add(Convolution2D(64, (3, 3), strides=(1, 1), padding='valid', activation='relu'))
    model.add(Flatten())
    model.add(Dropout(0.5))
    model.add(Dense(1164))
    model.add(Dense(100))
    model.add(Dense(50))
    model.add(Dense(10))
    model.add(Dense(1))
    return model

model = train_genarator_model(nvida_model(),
                              train_generator, train_steps,
                              validation_generator, validation_steps,
                              epochs=7)

nvda_eval = model.evaluate_generator(test_generator, len(test_samples)/generator_batch)
print(nvda_eval)

import datetime
now = datetime.datetime.now()
model_name = 'track2_nvda_{}_{}.h5'.format(now.hour, now.minute)
print(model_name)
model.save(model_name)