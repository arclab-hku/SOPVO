import time, sys, os
from ros import rosbag
import roslib
import rospy
roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import Image

# import ImageFile
from PIL import ImageFile
from PIL import Image as ImagePIL

'''sort image name'''
def CompSortFileNamesNr(f):
    g = os.path.splitext(os.path.split(f)[1])[0]
    numbertext = ''.join(c for c in g if c.isdigit())
    return int(numbertext)

'''get image from dir'''
def GetFilesFromDir(dir):
    '''Generates a list of files from the directory'''
    print( "Searching directory %s" % dir )
    all = []
    left_files = []
    right_files = []
    if os.path.exists(dir):
        for path, names, files in os.walk(dir):
            # for f in files:
            for f in sorted(files, key=CompSortFileNamesNr):
                if os.path.splitext(f)[1] in ['.bmp', '.png', '.jpg']:
                    if 'left' in f or 'left' in path:
                        left_files.append( os.path.join( path, f ) )
                    elif 'right' in f or 'right' in path:
                        right_files.append( os.path.join( path, f ) )
                    all.append( os.path.join( path, f ) )
    return all, left_files, right_files

def CreateMonoBag(imgs, bagname, timestamps):
    
    '''read time stamps'''
    file = open(timestamps, 'r')
    timestampslines = file.readlines()
    file.close()
    '''Creates a bag file with camera images'''
    bag =rosbag.Bag(bagname, 'w')

    try:
        for i in range(len(imgs)):
            print("Adding %s" % imgs[i])
            fp = open( imgs[i], "r" )
            p = ImageFile.Parser()

            '''read image size'''
            imgpil = ImagePIL.open(imgs[0])
            width, height = imgpil.size
            # print "size:",width,height
            # width 1241, height 376

            while 1:
                s = fp.read(1024)
                if not s:
                    break
                p.feed(s)

            im = p.close()

            Stamp = rospy.rostime.Time.from_sec(float(timestampslines[i]))

            '''set image information '''
            Img = Image()
            Img.header.stamp = Stamp
            Img.height = height
            Img.width = width
            Img.header.frame_id = "camera"

            '''for rgb8'''
            # Img.encoding = "rgb8"
            # Img_data = [pix for pixdata in im.getdata() for pix in pixdata]
            # Img.step = Img.width * 3

            '''for mono8'''
            Img.encoding = "mono8"
            Img_data = [pix for pixdata in [im.getdata()] for pix in pixdata]
            Img.step = Img.width

            Img.data = Img_data
            bag.write('camera/image_raw', Img, Stamp)
    finally:
        bag.close()       


def CreateBag(args):
    '''Creates the actual bag file by successively adding images'''
    all_imgs, left_imgs, right_imgs = GetFilesFromDir(args[0])
    if len(all_imgs) <= 0:
        print("No images found in %s" % args[0])
        exit()

    if len(left_imgs) > 0 and len(right_imgs) > 0:
        # create bagfile with stereo camera image pairs
        CreateStereoBag(left_imgs, right_imgs, args[1])
    else:
        # create bagfile with mono camera image stream
        CreateMonoBag(all_imgs, args[1], args[2])        

if __name__ == "__main__":
    if len( sys.argv ) == 4:
        CreateBag(sys.argv[1:])
    else:
        print( "Usage: img2bag_kitti_odo.py imagedir bagfilename timestamp")
