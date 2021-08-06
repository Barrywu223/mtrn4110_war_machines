import cv2
image = cv2.imread('../black_and_white.png')
image = cv2.resize(image, [900, 500])
img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)


ret, thresh = cv2.threshold(img_gray, 150, 255, cv2.THRESH_BINARY)

# visualize the binary image

cv2.imshow('Binary image', thresh)

cv2.waitKey(0)

cv2.imwrite('image_thres1.jpg', thresh)

cv2.destroyAllWindows()


contours, hierarchy = cv2.findContours(
    image=thresh, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)

image_copy = image.copy()

cv2.drawContours(image=image_copy, contours=contours, contourIdx=-1,
                 color=(0, 255, 0), thickness=-1)

cv2.imshow('None approximation', image_copy)

cv2.waitKey(0)

cv2.imwrite('contours_none_image1.jpg', image_copy)

cv2.destroyAllWindows()
