import os,cv2
import numpy as np
import sys
from keras.models import load_model
from keras import backend as K
K.set_image_dim_ordering('th')
import selectivesearch
import keras

# Initialization
img_rows = 32
img_cols = 32
num_channel = 3
batch_size = 256
#file_name = 'model'

#os.chdir(os.path.dirname(sys.argv[0]))
#path = os.getcwd()+'/'

# load model
model = load_model('model.h5')
print("Loaded model from disk") 

optimizer = keras.optimizers.RMSprop(lr=0.0001, rho=0.9, epsilon=1e-08, decay=0.0)

model.compile(optimizer = optimizer , loss = "categorical_crossentropy", metrics=["accuracy"])
'''
image_list = []
for filename in glob.glob(os.getcwd()+'/predict/*'): #assuming gif
		input_img=cv2.imread(filename)
		image_list.append(input_img)						
'''		
scale = 2000
sigma = 1.1
min_size = 500
#img = cv2.imread(path+'1.jpg')
img = cv2.imread('street.jpg')
img_lbl, regions = selectivesearch.selective_search(img , scale=scale, sigma=sigma, min_size=min_size)
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

elements = []
count = 0

for x, y, w, h in candidates:
    print(x, y, w, h)
    crop_img = img[y:y+h, x:x+w].copy() 
    crop_img_resize = cv2.resize(crop_img,(img_rows,img_cols))
    #cv2.imshow("cropped", crop_img_resize)
    img_data = np.array(crop_img_resize)
    img_data = img_data.astype('float32')
    img_data /= 255
	
    #print(img_data.shape)
	
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
    
    img_data = img_data.reshape(1, num_channel, img_rows, img_cols)
    prediction = model.predict(img_data, batch_size=1)
    classes = model.predict_classes(img_data, batch_size=1)
    proba = model.predict_proba(img_data, batch_size=1)
	
    predict = np.around(prediction, decimals=1)
    predict = predict.flatten()
    result = list(predict)
    #result = list(np.around(np.array(prediction[i]),2))
    print(result)
    for j in result:
        print(j)
        if (j >= 0.7):
            print('ide')
            elements.append([])
            elements[count].append(x)
            elements[count].append(y)
            elements[count].append(w)
            elements[count].append(h)
            elements[count].append(int(classes))
            elements[count].append(result.index(j))
            #elements[count].extend(x,y,w,h,int(classes), result.index(j))
            count += 1
            print(classes)
            break
	
	
    print(predict)
    print(classes)
    cv2.imshow('image',crop_img)
    if cv2.waitKey(1000000) == ord('q'):
        cv2.destroyWindow('image')
        break
    else:
        cv2.destroyWindow('image')
        continue

img2 = img.copy()
for x, y, w, h in candidates:
        print(x, y, w, h)
        cv2.rectangle(img2,(x,y),(x+w,y+h),(0,255,0),2)
		
cv2.imshow('image',img2)
cv2.waitKey(100000)	
cv2.destroyWindow('image')

for i in elements:
    x = i[0]
    y = i[1]
    w = i[2]
    h = i[3]
    r = i[4]
    cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
    print(r)

cv2.imshow('image',img)
cv2.waitKey(100000)
