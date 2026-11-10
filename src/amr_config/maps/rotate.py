from PIL import Image

img = Image.open('CCL.pgm')
rotated = img.rotate(-0.2)
rotated.save('CCL__.pgm')