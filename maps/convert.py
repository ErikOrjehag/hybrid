from cv2 import imread, imwrite, threshold, THRESH_BINARY

image_path = './'
image_name = 'mymap_B14'
image_modify_name = 'mymap_B14_modify'
image =  imread(image_path+image_name+'.png')
thresh = 128
image_binary =  threshold(image, thresh, 255,  THRESH_BINARY)[1]
imwrite(image_path+ image_modify_name+ '0.png' ,image_binary)