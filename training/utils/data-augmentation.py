#this script generate 5 new images for each image of the tin can's training set (of Open Images) by translating each image of a percentage of its lenght (2%, 4%, 6%, 8%, 10%)



from PIL import Image
import matplotlib.pyplot as plt
import os
import pandas as pd
from copy import copy

def roll(image, delta):
    """Roll an image sideways and append black pixels on the right."""
    xsize, ysize = image.size

    delta = int(delta*xsize)

    delta = delta % xsize
    if delta == 0: return image

    new_im = Image.new("RGB", (xsize, ysize))
    part2 = image.crop((delta, 0, xsize, ysize))
    new_im.paste(part2, (0, 0, xsize-delta, ysize))
    #part1 = image.crop((0, 0, delta, ysize))		#if you want to paste the cut part on the right instead of black pixels
    #image.paste(part1, (xsize-delta, 0, xsize, ysize))
    #return image

    return new_im

annotation = pd.read_csv("annotations.csv")		#path to the file with annotations downloaded from Open Images es. "train-annotations-bbox.csv"
total_row = annotation.shape[0]

print("begin")

for numer, row in annotation.iterrows():
  print(numer, total_row)
  if row['LabelName'] == "/m/02jnhm": 			#this is the string for tin cans in OpenImages, change it to change the object of interest
    for i in range(5): 					#change 5 if you want more or less images
      to_append = copy(row)
      to_append['ImageID'] = str(i) + to_append['ImageID']
      to_append['XMin'] -=(i+1)*0.02
      to_append['XMax'] -=(i+1)*0.02

      if to_append['XMin'] > 0:
        annotation = annotation.append(to_append, ignore_index=True)
      elif to_append['XMin'] < 0 and to_append['XMax'] > 0:
        to_append['XMin'] = 0
        annotation = annotation.append(to_append, ignore_index=True)
      #elif to_append['XMax'] < 0:						#if you want to paste the cut part of the image to the right
      #  to_append['XMin'] = 1 + to_append['XMin']				#use this part to check if the tin can has been split in two parts
      #  to_append['XMax'] = 1 + to_append['XMax']
      #  annotation = annotation.append(to_append, ignore_index=True)
      #elif to_append['XMin'] < 0 and to_append['XMin'] > -0.09:
      #  to_append['XMin'] = 0
      #  annotation = annotation.append(to_append, ignore_index=True)
      #else:
      #  new_to_append = copy(to_append)
      #  to_append['XMin'] = 0
      #  annotation = annotation.append(to_append, ignore_index=True)
      #  new_to_append['XMin'] = 1 + new_to_append['XMin']
      #  new_to_append['XMax'] = 1
      #  annotation = annotation.append(new_to_append, ignore_index=True)

annotation.sort_values('ImageID' , inplace=True)
annotation.to_csv("annotations-da-black-border.csv", index = False)


print("middle")

for image in os.listdir("images"):				#directory where images to augment are saved
    f = os.path.join("images", image)
    # checking if it is a file
    if os.path.isfile(f):
      for i in range(5):
        im = Image.open(os.path.join("images", image))
        im2 = roll(im, (i+1)*0.02)
        im2.save("images/" + str(i) + image)

print("end of script")