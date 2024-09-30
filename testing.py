import numpy as np
import os
import matplotlib.pyplot as plt
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.models import load_model

model = load_model(r'C:\Users\Lenovo\Desktop\cim\gender\Dataset\gender_classification_model.h5')

Test_path = r'C:\Users\Lenovo\Desktop\cim\gender\Dataset\Test'

test_datagen = ImageDataGenerator(rescale=1. / 255)

test_gen = test_datagen.flow_from_directory(
    Test_path,
    target_size=(250, 250),
    batch_size=32,
    class_mode='binary',
    shuffle=False
)

test_loss, test_acc = model.evaluate(test_gen, steps=test_gen.samples // test_gen.batch_size)
print(f'Test Accuracy: {test_acc * 100:.2f}%')

# Make predictions on the test dataset
predictions = model.predict(test_gen)
predicted_classes = (predictions > 0.5).astype(int)  # Convert probabilities to binary

for i in range(100):
    img = test_gen[i][0][0]
    plt.imshow(img)
    plt.title(f'Predicted: {"Female" if predicted_classes[i][0] == 0 else "Male"}')
    plt.axis('off')
    plt.show()
