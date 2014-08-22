<html>
<head>
<?cs include:"includes.cs" ?>
    <script type="text/javascript" src="jslib/camera_app.js"></script>
</head>

<body>
<?cs include:"header.cs" ?>
<h3>Camera viewer</h3>

<h4>Look through the various eyes of your PR2.</h4>

<table>
<tr>
<td><img id="left_image" src=""/></td>
<td><img id="right_image" src=""/></td>
</tr>
<tr>
<td>Select camera: <select onchange="left_image_select()" id="left_image_select"></select></td>
<td>Select camera: <select onchange="right_image_select()" id="right_image_select"></select></td>
</tr>
</table>

<div id=ErrorDiv></div>


<?cs include:"ros_app_include_footer.cs" ?>
</body>
</html>
