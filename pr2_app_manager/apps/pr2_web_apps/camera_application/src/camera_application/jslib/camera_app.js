var camera_app_urlprefix;
var camera_app_urlsuffix;
var camera_names;

jQuery(document).ready(function() {
  camera_app_urlprefix = gPump.hostport_prefix + '/image_stream?topic=/';
  camera_app_urlsuffix = '/image_rect_color_throttle';
  // TODO: get these topic names from somewhere
  camera_names = ['wide_stereo/left', 'wide_stereo/right', 'narrow_stereo/left', 'narrow_stereo/right', 'l_forearm_cam', 'r_forearm_cam'];

  stuff_select_list('#left_image_select');
  stuff_select_list('#right_image_select');

  if(!jQuery.cookie('left_image'))
    jQuery.cookie('left_image', camera_names[0]);
  if(!jQuery.cookie('right_image'))
    jQuery.cookie('right_image', camera_names[1]);

  jQuery('#left_image_select').val(jQuery.cookie('left_image'));
  jQuery('#right_image_select').val(jQuery.cookie('right_image'));

  left_image_select();
  right_image_select();
});

function left_image_select()
{
  var name = jQuery('#left_image_select :selected').val();
  var url = camera_app_urlprefix + name + camera_app_urlsuffix;
  jQuery("#left_image")[0].src = url;
  jQuery.cookie('left_image', name);
}

function right_image_select()
{
  var name = jQuery('#right_image_select :selected').val();
  var url = camera_app_urlprefix + name + camera_app_urlsuffix;
  jQuery("#right_image")[0].src = url;
  jQuery.cookie('right_image', name);
}

function stuff_select_list(selector)
{
  for(var i=0; i < camera_names.length; ++i){
    jQuery(selector).get(0).options[jQuery(selector).get(0).options.length] = new Option(camera_names[i], camera_names[i]);
  }
}
