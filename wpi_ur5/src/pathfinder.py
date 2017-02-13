def main():
    print 'Please select an image '
    im1=raw_input('Enter your file path ')
    print 'Is the object solid or wireframe? '
    ans=raw_input('Enter: ')
    if ans=='solid':
        edge(im1)
    elif ans=='wireframe':
        skeleton(im1)
    else:
        print 'I did not understand'
        sys.exit()

def skeleton(picture):
    # Generate the data
    import matplotlib.pyplot as plt
    from skimage.morphology import skeletonize, skeletonize_3d
    from skimage.data import binary_blobs
    import cv2
    import math

    from skimage.morphology import medial_axis, skeletonize, skeletonize_3d
    from skimage.filters import threshold_otsu
    from scipy import ndimage
    import numpy as np
    image = cv2.imread(picture, cv2.CV_LOAD_IMAGE_GRAYSCALE)
    height, width = image.shape
    ret, data = cv2.threshold(image, 127, 255, cv2.THRESH_BINARY)

    detector = cv2.SimpleBlobDetector()

    thresh = threshold_otsu(image)
    data = image < thresh

    # Compute the medial axis (skeleton) and the distance transform
    skel, distance = medial_axis(data, return_distance=True)

    # Compare with other skeletonization algorithms
    skeleton = skeletonize(data)
    skeleton3d = skeletonize_3d(data)
    pixels = np.where(skeleton3d == 255)
    i = 0;
    p = [];
    print pixels[0].size

    while i < pixels[0].size:
        p = p + [[pixels[1][i], (pixels[0][i])]];
        i=i+1;

    n=0;
    points=p;
    start = [points[0][0], points[0][1]];
    way=[start];
    current_p = [points[0][0], points[0][1]];
    number_neighbors=[];

    del points[0]
    while len(points)>0:
        i=0;
        d =[];
        while i<len(points):
            new_p=points[i];
            dy = new_p[1]-current_p[1];
            dx = new_p[0]-current_p[0];
            d = d+[math.sqrt(dx**2+dy**2)];
            i = i+1;
        way=way+[points[d.index(min(d))]];
        current_p = points[d.index(min(d))];
        del points[d.index(min(d))];


    from sklearn.neighbors import KDTree
    import numpy as np
    X = way;
    kdt = KDTree(X, leaf_size=30, metric='euclidean')
    kdt.query(X, k=10, return_distance=False)
    print X

    for i in range(len(X)):
        X[i][1]=X[i][1]*(-1)+height;

    import matplotlib.pyplot as plt
    from matplotlib.path import Path
    import matplotlib.patches as patches
    start=Path.MOVETO;
    rest=[start];

    for i in xrange(1,len(X)):
        rest=rest+[Path.LINETO]

    path = Path(X, rest)



    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_xlim(0,width)
    ax.set_ylim(0,height)
    patch = patches.PathPatch(path, facecolor='white', lw=2, ls='dashed', edgecolor='green')
    ax.add_patch(patch)
    plt.show(patch)


    '''
    # Distance to the background for pixels of the skeleton
    dist_on_skel = distance * skel

    fig, axes = plt.subplots(2, 2, figsize=(8, 8), sharex=True, sharey=True,
                             subplot_kw={'adjustable': 'box-forced'})
    ax = axes.ravel()

    ax[0].imshow(data, cmap=plt.cm.gray, interpolation='nearest')
    ax[0].set_title('input data')
    ax[0].axis('off')

    ax[1].imshow(dist_on_skel, cmap=plt.cm.spectral, interpolation='nearest')
    ax[1].contour(data, [0.5], colors='w')
    ax[1].set_title('medial_axis')
    ax[1].axis('off')

    ax[2].imshow(skeleton, cmap=plt.cm.gray, interpolation='nearest')
    ax[2].set_title('skeletonize')
    ax[2].axis('off')

    ax[3].imshow(skeleton3d, cmap=plt.cm.gray, interpolation='nearest')
    ax[3].set_title('skeletonize_3d')
    ax[3].axis('off')

    fig.tight_layout()
    plt.show()
    '''

def edge(picture):
    import cv2
    import numpy as np
    from matplotlib import pyplot as plt
    import math

    img = cv2.imread(picture, 0)
    edges = cv2.Canny(img, 100, 200)
    ret, data = cv2.threshold(edges, 127, 255, cv2.THRESH_BINARY)
    '''cv2.imshow('pic', data)
    plt.subplot(121), plt.imshow(img, cmap='gray')
    plt.title('Original Image'), plt.xticks([]), plt.yticks([])
    plt.subplot(122), plt.imshow(edges, cmap='gray')
    plt.title('Edge Image'), plt.xticks([]), plt.yticks([])'''

    height, width = img.shape

    pixels = np.where(data == 255)
    i = 0;
    p = [];
    print pixels[0].size

    while i < pixels[0].size:
        p = p + [[pixels[1][i], (pixels[0][i])]];
        i = i + 1;

    n = 0;
    points = p;
    start = [points[0][0], points[0][1]];
    way = [start];
    current_p = [points[0][0], points[0][1]];
    number_neighbors = [];

    del points[0]
    while len(points) > 0:
        i = 0;
        d = [];
        while i < len(points):
            new_p = points[i];
            dy = new_p[1] - current_p[1];
            dx = new_p[0] - current_p[0];
            d = d + [math.sqrt(dx ** 2 + dy ** 2)];
            i = i + 1;
        way = way + [points[d.index(min(d))]];
        current_p = points[d.index(min(d))];
        del points[d.index(min(d))];

    from sklearn.neighbors import KDTree
    import numpy as np
    X = way;
    kdt = KDTree(X, leaf_size=30, metric='euclidean')
    kdt.query(X, k=10, return_distance=False)
    print X


    for i in range(len(X)):
        X[i][1] = X[i][1] * (-1) + height;

    import matplotlib.pyplot as plt
    from matplotlib.path import Path
    import matplotlib.patches as patches
    start = Path.MOVETO;
    rest = [start];

    for i in xrange(1, len(X)):
        rest = rest + [Path.LINETO]

    #Used to close the shape
    rest=rest+[Path.CLOSEPOLY]
    X.append(X[0])


    path = Path(X, rest)

    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_xlim(0, width)
    ax.set_ylim(0, height)
    patch = patches.PathPatch(path, facecolor='white', lw=2, ls='dashed', edgecolor='green')
    ax.add_patch(patch)
    plt.show(patch)

main()