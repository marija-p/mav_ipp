img_seg_sub = rossubscriber('/firefly/image_seg');
pause(1);

i = 0;

while(true)
    img_seg_msg = receive(img_seg_sub);
    disp('Received an image!');
    img_seg = readImage(img_seg_msg);
    imwrite(img_seg, ['image', num2str(i,'%04d'), '.png']);
    i = i + 1;
end