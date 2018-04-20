import os,cv2
import numpy as np
import glob
import sys
from keras.models import load_model
from keras import backend as K
K.set_image_dim_ordering('th')
import keras
import selectivesearch
import errno
import cv

global model

FIFO_R = '/tmp/fifo'
FIFO_W = '/tmp/fifo1'
image = []
timeStamp = ""
result = ""
delim = "break"

img_rows = 32
img_cols = 32
num_channel = 3

#os.chdir(os.path.dirname(sys.argv[0])+'/')

filename = 'model.h5'
model = load_model(filename)

optimizer = keras.optimizers.RMSprop(lr=0.0001, rho=0.9, epsilon=1e-08, decay=0.0)
model.compile(optimizer = optimizer , loss = "categorical_crossentropy", metrics=["accuracy"])

#definovanie funkcie na citanie z pajpy
def readImageFromPipe():
    #overenie ci pajpa existuje
    try:
        os.mkfifo(FIFO_R)
    except OSError as oe: 
        if oe.errno != errno.EEXIST:
            raise
    
    #otvorenie pajpy FIFO_R
    print("Opening FIFO...")
    with open(FIFO_R) as fifo:
        print("FIFO opened")
        
        #rozdelenie dat podla kluca delim
        data = fifo.read().split(delim)
        w = int(data[0])
        h = int(data[1])
        d = int(data[2])
        timeStamp = data[3]
        imageData = data[4]
                
        print(len(timeStamp))
        print(len(imageData))

        #vytvorenie numpyarray z precitanych dat
        nparr = np.fromstring(imageData, np.uint8)

        #vytvarovanie obrazka 
        print(nparr.size)
        image=nparr.reshape(h,w,d)
        return timeStamp, image

#definovanie funkcie na zapis udajov do pajpy
def writeImageToPipe(timeStamp, result):
    try: 
        os.mkfifo(FIFO_W)
    except OSError as oe:
        if oe.errno != errno.EEXIST:
            raise
    #otvorenie pajpy na zapis
    with open(FIFO_W, "w") as fifo:
        #treba oddelit jednotlive casti symbolom \n aby sa v c++ dalo nacitat cez getline
        fifo.write(timeStamp + "\n" + result + "\n")
    fifo.close()
    #sleep(.01)
    return

def format_image(image):
    img = cv2.resize(image,(img_rows,img_cols))
    img_data = np.array(img)
    img_data = img_data.astype('float32')
    img_data /= 255

    if num_channel==1:
        if K.image_dim_ordering()=='th':
            img_data = np.expand_dims(img_data, axis=0)
            img_data = np.expand_dims(img_data, axis=0)
        else:
            img_data = np.expand_dims(img_data, axis=3) 
            img_data = np.expand_dims(img_data, axis=0)
		
    else:
        if K.image_dim_ordering()=='th':
            img_data = np.rollaxis(img_data,2,0)
            img_data= np.expand_dims(img_data, axis=0)
        else:
            img_data= np.expand_dims(img_data, axis=0)
    
    img_data = img_data.reshape(img_data.shape[0], num_channel, img_rows, img_cols)

    return img_data
	
def predict_class(image):
    prediction = model.predict(image, batch_size=1)
    classes = model.predict_classes(image, batch_size=1)
    proba = model.predict_proba(image, batch_size=1)
    predict = np.around(prediction, decimals=1)
    predict = predict.flatten()
    predict = list(predict)
    for i in predict:
        if i >= 0.7:
            return i
    return -1
	

def selective_search(image):	
    result = []
    i = 0
    image = image[:, :, :3]
    print image.shape
    height, width, channels = image.shape
    scale = 3*((height > width) and height or width)
    sigma = 1.1
    min_size = (height > width) and height or width

    img_lbl, regions = selectivesearch.selective_search(image , scale=300, sigma=0.9, min_size=10)
    candidates = set()
    for r in regions:
        if r['rect'] in candidates:
            continue
        if r['size'] < 2000:
            continue
        x, y, w, h = r['rect']
        if w / h > 1.2 or h / w > 1.2:
            continue
        candidates.add(r['rect'])
    
    for x, y, w, h in candidates:
	
        crop_img = image[y:y+h, x:x+w]
        crop_img = format_image(crop_img)
        classes = predict_class(crop_img)
        if(classes > 0):
            result.append([])
            result[i].append(x)
            result[i].append(y)
            result[i].append(w)
            result[i].append(h)			
            result[i].append(int(classes))
            i += 1

    return result

def predict_image(classes):	
    classes = []
    for i in range (0,20):
        for j in classes:
            if(j[4] == i):
                classes.append(i)
                break
    return classes
	
	
	
def main():
    result = []

    while(True):
        timeStamp, image = readImageFromPipe()
        #cv2.imshow(timeStamp, image)
        #cv2.waitKey(10000)        

        #resize = cv2.resize(image, (320, 240), interpolation = cv2.INTER_LINEAR) 
        result = selective_search(image)
        
        for i in result:
            x = i[0]
            y = i[1]
            w = i[2]
            h = i[3]
            r = i[4]
            #print(r)
            cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,0),2) 
        
        #image = cv2.resize(image, (320, 240), interpolation = cv2.INTER_LINEAR)  
        #resize = cv2.resize(image, (320, 240), interpolation = cv2.INTER_LINEAR)  
        cv2.imshow('frame',image)

        #zapis do pajpy FIFO_W
        writeImageToPipe(timeStamp, result)
        #print(result)
	
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
	
if __name__ == "__main__":
    main()
