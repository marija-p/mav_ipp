process_img_srv = rossvcclient('/firefly/process_image');
process_img_req = rosmessage(process_img_srv);

process_img_srv = call(process_img_srv, process_img_req, 'Timeout', 100);