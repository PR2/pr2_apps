<html>
  <head>
    <?cs include:"includes.cs" ?>
    <link rel="stylesheet" type="text/css" href="templates/style_desktop.css">
    <?cs if:CGI.cur.device_style ?>
      <link rel="stylesheet" type="text/css" href="<?cs var:CGI.ScriptName?>/webui/templates/css/<?cs var:CGI.cur.device_style?>">
    <?cs /if ?>
    <script>
      document.listen_only = <?cs var:CGI.listen_only?>
    </script>
    <script type="text/javascript" src="jslib/map_viewer.js"></script>
  </head>

  <body>
    <?cs include:"header.cs" ?>
    <h3>Robot Navigation Application</h3>

    <div class=mapviewer objtype=MapViewer>
<!--
        <div id="button_bar" style="padding: 5px; z-index: 1; position: absolute;">
        <img src="/webui/app/navigation_application/navigation_stage/images/zoomin_disable.png" title="Zoom In" id="zoomin_button" style="border: 2px solid black;"/><br/>
        <img src="/webui/app/navigation_application/navigation_stage/images/zoomout_disable.png" title="Zoom Out" id="zoomout_button" style="border: 2px solid black;"/><br/>
        <img src="/webui/app/navigation_application/navigation_stage/images/reset_disable.png" title="Reset the view" id="reset_button" style="border: 2px solid black;"/><br/><br/>
        <img src="/webui/app/navigation_application/navigation_stage/images/pan_disable.png" title="Pan" id="pan_button" style="border: 2px solid rgb(68, 255, 68);"/><br/>
        <img src="/webui/app/navigation_application/navigation_stage/images/goal_disable.png" title="Set the robot's goal" id="goal_button" style="border: 2px solid black;"/><br/>
        <img src="/webui/app/navigation_application/navigation_stage/images/pose_disable.png" title="Set the robot's initial pose" id="pose_button" style="border: 2px solid black;"/><br/><br/>
        <img src="/webui/app/navigation_application/navigation_stage/images/pin_disable.png" title="Follow the robot" id="pin_button" style="border: 2px solid black;"/><br/>
        </div>
-->
        <div class="app_help_link app_link">
            <a href="" id="show_help">&laquo; Show Help</a>
        </div>
        <div class="app_instructions">
            <div class="app_instructions_box">
                <h3>
                    Welcome to the Robot Navigation Application!
                </h3>
                <p>
                    Here's how to make the robot go somewhere:
                    <ol>
                        <li>Tell the robot where it is
                        <p>
                            A. Click the "Set Pose" button:
                            <img src="/webui/app/navigation_application/navigation_stage/images/pose_disable.png" id="instructions_set_pose" title="Set the robot's initial pose" style=""/>
                        </p>
                        <p>
                            B. Click on the map at the robot's <strong>current actual location</strong>.  Be sure to rotate the robot icon with your mouse to match the
                            robot's current orientation.
                        </p>
                        <li>Tell the robot where you want it to go
                        <p>
                            A. Click the "Set Goal" button:
                            <img src="/webui/app/navigation_application/navigation_stage/images/goal_disable.png" id="instructions_set_goal" title="Set the robot's goal" style=""/>
                        </p>
                        <p>
                            B. Click on the map where you want the robot to go.  Be sure to rotate the robot icon with your mouse to match
                            your desired final orientation.
                        </p>
                    </ol>
                </p>
                <hr>
                <p>
                    Here's how to understand what you're seeing:
                    <ol>
                        <li>Green lines are scans from the robot's base laser.  
                        <p>
                            The green lines should match fairly well with obstacles in the map.  If they don't match, then the robot might not know where it is.  Try telling it where it is with the "Set Pose" button as described above.
                        </p>
                        <li>Red areas are obstacles that the robot sees.
                        <p>
                            No part of the robot is allowed to drive over the red areas.
                        </p>
                        <li>Blue areas are the safety margin around obstacles that the robot sees.
                        <p>
                            The center of the robot is not allowed to drive over the blue areas.
                        </p>
                    </ol>
                </p>
                <div class="app_link">
                    <a href="" id="hide_help">&raquo; Hide Help</a>
                </div>
            </div>
        </div>
    </div>

    <div class="divclear"></div>

    <script>
        jQuery(document).ready(function() {
            
            jQuery("#show_help").click(function() {
                jQuery('#map_status').hide();
                jQuery('.app_help_link').slideUp();
                jQuery('.app_instructions').slideDown();
                return false;
            });

            jQuery("#hide_help").click(function() {
                jQuery('#map_status').show();
                jQuery('.app_instructions').slideUp();
                jQuery('.app_help_link').slideDown();
                
                // don't show this by default for this user anymore
                jQuery.get("/webui/active/active.py?Action.DismissNotice=1&notice=nav_app");
                return false;
            });
            
            jQuery("#instructions_set_pose").click(function() {
                gPump.widget_hash["MapViewer"].setPoseMode();
                jQuery('.app_instructions').slideUp();
                jQuery('.app_help_link').slideDown();
                return false;
            });

            jQuery("#instructions_set_goal").click(function() {
                gPump.widget_hash["MapViewer"].setGoalMode();
                jQuery('.app_instructions').slideUp();
                jQuery('.app_help_link').slideDown();
                return false;
            });
            
            jQuery(".app_instructions").bind("mouseup mousedown click", function() {
                return false;
            });

            jQuery("#show_help").bind("mouseup mousedown click", function() {
                return false;
            });
             
            jQuery.get("/webui/active/active.py?Action.NoticeDismissed=1&notice=nav_app", function(ret_val) {
                if (ret_val.indexOf("YES") > -1) {
                    jQuery('.app_help_link').slideDown();
                    showMapStatus();
                } else {
                    jQuery('.app_instructions').slideDown();
                    jQuery('#map_status').hide();
                }
            });
        });
    </script>

<?cs include:"ros_app_include_footer.cs" ?>

