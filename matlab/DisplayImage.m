function DisplayImage

global im_pts camera_real

title('position of target in current image');
plot_image(im_pts, camera_real, 'r*')

legend('target');