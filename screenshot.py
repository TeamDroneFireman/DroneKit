import os
import gtk.gdk
import time
import argparse  
parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')

parser.add_argument('--fichier', 
                   help="le nom du fichier kml")
args = parser.parse_args()

f=(str)(args.fichier)

#os.system('rm -r /home/nounou/.googleearth')
os.system('xdg-open '+f)
time.sleep(12)
#os.system('xdg-open')
w = gtk.gdk.get_default_root_window()
sz = w.get_size()
print "The size of the window is %d x %d" % sz
pb = gtk.gdk.Pixbuf(gtk.gdk.COLORSPACE_RGB,False,8,sz[0],sz[1])
pb = pb.get_from_drawable(w,w.get_colormap(),0,0,0,0,sz[0],sz[1])
if (pb != None):
    pb.save("screenshot.png","png")
    print "Screenshot saved to screenshot.png."
else:
    print "Unable to get the screenshot."


