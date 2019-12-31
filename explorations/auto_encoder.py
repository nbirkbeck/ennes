from keras.layers import Input, Dense
from keras.models import Model
from keras.datasets import mnist
from keras import regularizers
from keras.callbacks import TensorBoard
from keras.layers import Input, Dense, Conv2D, MaxPooling2D, UpSampling2D, GlobalAveragePooling2D

import numpy as np
import matplotlib.pyplot as plt

def normalize_data(x, cnn):
    if cnn:
        return (x.astype('float32') / 255.).reshape((len(x), 28, 28, 1))
    else:
        return (x.astype('float32') / 255.).reshape((len(x), np.prod(x.shape[1:])))
    
def load_data(cnn):
    (x_train, _), (x_test, _) = mnist.load_data()
    return normalize_data(x_train, cnn), normalize_data(x_test, cnn)

cnn = False
x_train, x_test = load_data(cnn)

encoding_dim = 2


def build_multilayer_model(input_img, encoding_dim):
    encoded = Dense(128, activation='relu')(input_img)
    encoded = Dense(64, activation='relu')(encoded)
    encoded = Dense(encoding_dim, activation='relu', activity_regularizer=regularizers.l1(10e-7))(encoded)

    decoded = Dense(64, activation='relu')(encoded)
    decoded = Dense(128, activation='relu')(decoded)
    decoded = Dense(784, activation='sigmoid')(decoded)
    return Model(input_img, decoded)

def build_cnn(input_img, encoding_dim):
    x = Conv2D(8, (3, 1), activation='relu', padding='same')(input_img)
    x = Conv2D(8, (1, 3), activation='relu', padding='same')(x)
    x = MaxPooling2D((2, 2), padding='same')(x)
    x = Conv2D(4, (3, 1), activation='relu', padding='same')(x)
    x = Conv2D(4, (1, 3), activation='relu', padding='same')(x)
    x = MaxPooling2D((2, 2), padding='same')(x)
    x = Conv2D(4, (3, 1), activation='relu', padding='same')(x)
    x = Conv2D(4, (1, 3), activation='relu', padding='same')(x)
#    x = MaxPooling2D((2, 2), padding='same')(x)
#    x = Conv2D(2, (1, 1), activation='relu', padding='same')(x)
    encoded = MaxPooling2D((4,4), padding='same')(x)
#    x = UpSampling2D((2, 2))(encoded)
    x = Conv2D(8, (3, 3), activation='relu', padding='same')(x)
#    x = UpSampling2D((2, 2))(encoded)
#    x = Conv2D(1, (3, 3), activation='relu', padding='same')(x)
#    x = UpSampling2D((2, 2))(x)
#    x = Conv2D(4, (3, 3), activation='relu', padding='same')(x)
#    x = UpSampling2D((2, 2))(x)
#    x = Conv2D(4, (3, 3), activation='relu', padding='same')(x)
    x = UpSampling2D((2, 2))(x)
    x = Conv2D(8, (3, 3), activation='relu', padding='same')(x)
    x = UpSampling2D((2, 2))(x)
    decoded = Conv2D(1, (3, 3), activation='relu', padding='same')(x)
#    decoded = Conv2D(1, (3, 3), activation='relu', padding='valid')(x)
   # x = UpSampling2D((2, 2))(x)
   # decoded = Conv2D(1, (3, 3), activation='sigmoid', padding='same')(x)

    return Model(input_img, decoded)

if cnn:
    input_img = Input(shape=(28, 28, 1,))
    autoencoder = build_cnn(input_img, encoding_dim)
else:
    input_img = Input(shape=(28 * 28, ))
    autoencoder = build_multilayer_model(input_img, encoding_dim)

    
print(autoencoder.summary())
autoencoder.compile(optimizer='adadelta', loss='binary_crossentropy')

for k in range(0, 10):
    autoencoder.fit(x_train, x_train, epochs=2, batch_size=256,
                    validation_data=(x_test, x_test),
                    callbacks=[TensorBoard(log_dir='/tmp/autoencoder')])


    #encoder = Model(input_img, encoded)
    #encoded_input = Input(shape=(encoding_dim,))
    #decoder_layer = autoencoder.layers[-1]
    #decoder = Model(encoded_input, decoder_layer(encoded_input))

    decoded_imgs = autoencoder.predict(x_test) #decoder.predict(encoder.predict(x_test))
    print(decoded_imgs.shape)


    n = 10  # how many digits we will display
    plt.figure(figsize=(20, 4))
    for i in range(n):
        # display original
        ax = plt.subplot(2, n, i + 1)
        plt.imshow(x_test[i].reshape(28, 28))
        plt.gray()
        ax.get_xaxis().set_visible(False)
        ax.get_yaxis().set_visible(False)

        # display reconstruction
        ax = plt.subplot(2, n, i + 1 + n)
        plt.imshow(decoded_imgs[i].reshape(28, 28))
        plt.gray()
        ax.get_xaxis().set_visible(False)
        ax.get_yaxis().set_visible(False)
    plt.show()
