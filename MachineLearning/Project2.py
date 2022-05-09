import os
import matplotlib.pyplot as plt
#import scipy.ndimage as ndimage
from keras.models import Sequential
from keras.models import model_from_config
from keras.layers import Dense, Dropout, Activation, Flatten, Conv2D, MaxPooling2D
from sklearn.model_selection import train_test_split
import numpy as np
from os import listdir
from skimage import io
from skimage.transform import resize as imresize
import keras
from keras import activations
import tensorflow as tf
from tensorflow.keras.optimizers import Adam
from tensorflow.keras.utils import to_categorical as to_categorical

sz = 64
epoch = 50

tf.compat.v1.disable_eager_execution()

# Load Data

aquired_path = listdir(r"aquired")
x_aquired=[]
y_aquired = np.zeros(len(aquired_path))
not_aquired_path = listdir(r".\not_aquired")
x_not_aquired=[]
y_not_aquired = np.ones(len(not_aquired_path))

# aquired = 0 
# not_aquired = 1
for image in aquired_path:
    aquired_img = io.imread(str(r'aquired') + "/" + str(image))
    x_aquired.append(imresize(aquired_img, (sz, sz, 3)))

for image in not_aquired_path:
    not_aquired_img = io.imread(str(r'not_aquired') + "/" + str(image))
    x_not_aquired.append(imresize(not_aquired_img, (sz, sz, 3)))

x=np.asarray(x_aquired+x_not_aquired)
y=np.append(y_aquired,y_not_aquired)

# Keras
y = to_categorical(y)
train_x, test_x, train_y, test_y = train_test_split(x, y, test_size=0.1, random_state=30)

# Set model to sequential
input_shape=(sz, sz, 3)
model = Sequential()

# Add Convolution Layers
model.add(Conv2D(32, kernel_size = (3, 3),activation = 'relu', input_shape = input_shape)) 
model.add(Conv2D(64, (3, 3), activation = 'relu')) 
#model.add(MaxPooling2D(pool_size = (2, 2)))

# Flatten Conv Layer before passing to Dense Layer
model.add(Flatten())
model.add(Dense(2, name='test'))
model.add(Dense(2,activation='softmax', name='preds'))
adamop=Adam(lr=0.1)
 
model.compile(loss='binary_crossentropy', optimizer='adam', metrics=['accuracy'])
history = model.fit(train_x, train_y, validation_data=(test_x, test_y), batch_size=sz, epochs=epoch, shuffle=True, verbose=2)
score = model.evaluate(test_x, test_y)
pred = model.predict(test_x)
   
print('Test loss:', score[0])
print('Test accuracy:', score[1])
    
# summarize history for accuracy
plt.plot(history.history['accuracy'])
plt.plot(history.history['val_accuracy'])
plt.title('Model accuracy')
plt.ylabel('Accuracy')
plt.xlabel('Epoch')
plt.legend(['Train', 'Test'], loc='upper left')
plt.show()

# summarize history for loss
plt.plot(history.history['loss'])
plt.plot(history.history['val_loss'])
plt.title('Model loss')
plt.ylabel('Loss')
plt.xlabel('Epoch')
plt.legend(['Train', 'Test'], loc='upper left')
plt.show()

model.summary()
