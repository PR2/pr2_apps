<html>
<head>
<?cs include:"includes.cs" ?>
    <script type="text/javascript" src="jslib/plug_ui.js"></script>
</head>

<body>
<?cs include:"header.cs" ?>
<h3>PR2 Recharge</h3>

<h4>Use this application to request that your PR2 robot plug into or unplug from an electrical outlet. While the robot is plugging in, you might want to watch its progress with the camera or listen-only navigation applications.</h4>

<!-- This div allows the creation of the subscriber to recharge state, defined in plug_ui.js -->
<div objtype="PlugUI"></div>

<table>
<tr>
<td><b>Robot status</b>:</td>
<td><div id=PlugStatus></div></td>
</tr>
<tr>
<td><b>Current state of recharge task</b>:</td>
<td><div id=PlugSMStatus></div></td>
</tr>
</table>

<div class="outlet_selection">
<h4>Request a plug-in or unplug:</h4>
Select an oulet to plug into:
<select id="outlet_list">
 <option value="-1">Waiting on service</option>
</select>
</div>

<p>
<a href="" id="plug_button" onclick="plugIn(); return false;">&raquo; Plug In</a>
<a href="" id="unplug_button" onclick="unplug(); return false;">&raquo; Unplug</a>
</p>

<div id=ErrorDiv></div>

<hr>

<!--?cs include:"ros_app_include_footer.cs" ?-->

<!-- The following content is copied from webui/ros_app_include_footer.cs -->
    
    <?cs if:CGI.taskid ?>
        <a href="<?cs var:CGI.ScriptName?>/webui/apps.py" id="stop_recharge_app" onclick="stopRechargeApp(); return true;">&raquo; Stop this application</a>

        <!-- We hide the stop button and define our own, because I can't figure out how to hide the existing one. -->
        <!--div class="app_info" style="margin-top: 40px;" objtype="LaunchAppFromInfoPageWidget" taskid="<?cs var:CGI.taskid ?>" app_name="<?cs var:CGI.cur.app.name ?>">
          <p class="start_app">
            <a href="">&raquo; Start this application</a>
          </p>
          <p class="go_to_app">
            <p class="stop_app">
              <a href="">&raquo; Stop this application</a>
            </p>
          </p>
          <p>
            
          </p>
          <div class="dialog" style="display: none">
            The <?cs var:CGI.cur.app.name ?> app is now running.
          </div>
        </div-->

      <p class="divclear">
        &laquo; 
        <a href="<?cs var:CGI.ScriptName?>/webui/apps.py">All Apps</a>
        <!-- We hide the link to the app info page, to make it harder to stop the app in an unexpected place -->
      	<!--a href="<?cs var:CGI.ScriptName?>/webui/appinfo.py?taskid=<?cs var:CGI.taskid ?>">Application Info Page</a-->
      </p>
    <?cs /if ?>
    <div id=ErrorDiv></div>
    <?cs include:"rosfooter.cs"?>
  </body>
</html>
