import numpy as np
import os
import matplotlib.pyplot as plt
import matplotlib.image as mpimg  # For displaying images

import tensorflow as tf
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.layers import AveragePooling2D, Dense, Conv2D, Flatten, Dropout
from tensorflow.keras.models import Sequential

train_datagen = ImageDataGenerator(
    rescale=1. / 255,
    rotation_range=30,
    shear_range=0.3,
    zoom_range=0.3,n
)

test_datagen = ImageDataGenerator(rescale=1. / 255)

Train_path = r'C:\Users\Lenovo\Desktop\cim\gender\Dataset\Train'
Valid_path = r'C:\Users\Lenovo\Desktop\cim\gender\Dataset\Validation'

train_gen = train_datagen.flow_from_directory(
    Train_path,
    target_size=(250, 250),
    batch_size=32,
    class_mode='binary'  #  (male vs female)
)

# Creating the validation data generator
Valid_gen = test_datagen.flow_from_directory(
    Valid_path,
    target_size=(250, 250),
    batch_size=32,
    class_mode='binary'
)

model = Sequential()
model.add(Conv2D(32, kernel_size=(3, 3), activation='relu', input_shape=(250, 250, 3),
                 kernel_regularizer=tf.keras.regularizers.l2(0.001), padding="same"))
model.add(AveragePooling2D((2, 2)))

model.add(Conv2D(64, kernel_size=(3, 3), activation='relu'))
model.add(AveragePooling2D((2, 2)))
model.add(Conv2D(64, kernel_size=(3, 3), activation='relu'))
model.add(AveragePooling2D((2, 2)))

model.add(Conv2D(128, kernel_size=(3, 3), activation='relu'))
model.add(AveragePooling2D((2, 2)))
model.add(Conv2D(128, kernel_size=(3, 3), activation='relu'))
model.add(AveragePooling2D((2, 2)))

model.add(Flatten())
model.add(Dense(256, activation='relu'))
model.add(Dense(1, activation='sigmoid'))

model.summary()

model.compile(optimizer='rmsprop', loss='binary_crossentropy', metrics=['acc'])

"""history = model.fit(
    train_gen,
    steps_per_epoch=train_gen.samples // train_gen.batch_size,
    epochs=10,
    validation_data=Valid_gen,
    validation_steps=Valid_gen.samples // Valid_gen.batch_size
)"""
model.save('gender_classification_model.h5')

plt.plot(history.history['acc'], label='accuracy')
plt.plot(history.history['val_acc'], label='val_accuracy')
plt.xlabel('Epoch')
plt.ylabel('Accuracy')
plt.ylim([0, 1])
plt.legend(loc='lower right')
plt.show()
