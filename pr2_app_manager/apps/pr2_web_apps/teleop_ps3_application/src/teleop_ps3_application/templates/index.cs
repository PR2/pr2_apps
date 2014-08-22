<?cs include:"ros_app_include_header.cs"?>

<h3>PS3 Teleop Application</h3>

<img src="images/pairing.png" style="float: right; margin-left: 10px" />

<h4>1. Connecting the PS3 Controller to the Robot</h4>

<p>
  The PS3 controller must be "paired" with the robot.  If your controller is not already paired, you can pair it by pressing the center button, shown in the image on the right.
  <br />
</p>



<h4>2. Controlling the Robot with the PS3 Controller</h4>

<table>
<tr>
<th style="width: 120px">Button</th>
<th style="text-align: left">Control Action</th>
</tr>

<tr>
<th>L1</th>
<td>The "deadman" control;  The robot will not respond unless this button is held down</td>
<tr>

<tr>
<th>L2</th>
<td>The "head button"; allows the sticks to control the head instead of the base</td>
<tr>

<tr>
<th>Left stick</th>
<td>Rotate (yaw) the base</td>
</tr>

<tr>
<th>Right stick</th>
<td>Move (translate) the base</td>
</tr>

<tr>
<th>Triangle</th>
<td>Move the torso up</td>
</tr>

<tr>
<th>X</th>
<td>Move the torso down</td>
</tr>

<tr>
<th>R1</th>
<td>The "run" button; commands the base faster</td>
</tr>

</table>


<p>
Note: The <strong>L1</strong>, <strong>L2</strong>, <strong>R1</strong>, and <strong>R2</strong> buttons are all labeled on the controller.
</p>

<?cs include:"ros_app_include_footer.cs"?>

