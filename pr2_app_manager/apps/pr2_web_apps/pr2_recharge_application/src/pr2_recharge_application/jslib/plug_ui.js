jQuery(document).ready(function() { 
  plugui_onload();
  gPump.service_call2("outlet_locations", {}, 
  function(outlet_locations) {
    //clear the current list items
    jQuery("#outlet_list").get(0).options.length = 0;
    jQuery("#outlet_list").get(0).options[0] = new Option("Select an outlet", "-1");
    for(var i=0; i < outlet_locations.poses.length; ++i){
      jQuery("#outlet_list").get(0).options[jQuery("#outlet_list").get(0).options.length] = new Option(outlet_locations.poses[i].name, outlet_locations.poses[i].id);
    }
  });
});

var PlugUI = Class.create(Widget, {
  initialize: function($super, domobj) {
    $super(domobj);
    this.topics = ['/recharge_state', '/recharge/smach/container_status'];
    setStatus("Waiting to hear the state of recharge...");
    setLinkStates(false, false);
  },

  init: function() {
  },


  receive: function(topic, message) {
    if(topic == '/recharge_state')
    {
      if(message.state == 0)
      {
        setLinkStates(false, false);
        jQuery(".outlet_selection").hide();
        setStatus("Waiting to hear the state of recharge...");
      }
      else if(message.state == 1)
      {
        setLinkStates(true, false);
        jQuery(".outlet_selection").show();
        setStatus("The robot is unplugged");
      }
      else if(message.state == 2)
      {
        setLinkStates(false, true);
        jQuery(".outlet_selection").hide();
        setStatus("The robot is plugged in");
      }
      else
      {
        setLinkStates(false, false);
        jQuery(".outlet_selection").hide();
        jQuery("#stop_recharge_app").show();
        setStatus("The robot failed to plug in.  This is an unrecoverable error.  Please go find the robot, fix the problem, and reset the robot");
      }
    }
    else if(topic == '/recharge/smach/container_status')
    {
	if (message.path == '/RECHARGE')
        {
	    setSMStatus(message.active_states[0]);
	}
    }
  },

});

function setStatus(msg) {
  jQuery('#PlugStatus').html(msg);
}
function setSMStatus(msg) {
  jQuery('#PlugSMStatus').html(msg);
}

function setLinkStates(plug_state, unplug_state) {
  if(plug_state)
  {
    jQuery("#plug_button").show();
    jQuery("#stop_recharge_app").show();
  }
  else
  {
    jQuery("#plug_button").hide();
    jQuery("#stop_recharge_app").hide();
  }
  if(unplug_state)
  {
    jQuery("#unplug_button").show();
  }
  else
  {
    jQuery("#unplug_button").hide();
  }
}

function plugIn() {
  //gPump.publish("/recharge_command", "pr2_plugs_msgs/RechargeCommand", [2, "phase_space"]);
  if(jQuery("#outlet_list :selected").val() == -1){
    dialogMessage("Selection required", "You must select an outlet from the list to plug in.");
  }
  else{
    //ros_debug(jQuery("#outlet_list option:selected").val() + " " + jQuery("#outlet_list option:selected").text());
    gPump.service_call2("recharge_request", {'command': {'command': 2, 'plug_id': jQuery("#outlet_list option:selected").text()}});
    setStatus("Waiting to hear the state of recharge...");
    setLinkStates(false, false);
  }
}

function unplug() {
  //gPump.publish("/recharge_command", "pr2_plugs_msgs/RechargeCommand", [1, "phase_space"]);
  gPump.service_call2("recharge_request", {'command': {'command': 1, 'plug_id': ""}});
  setStatus("Waiting to hear the state of recharge...");
  setLinkStates(false, false);
}

function stopRechargeApp() {
  gPump.service_call("stop_task", {'taskid':'pr2_recharge_application/pr2_recharge_application', 'username':'anonymous'});
}

function plugui_onload() {
  //ros_debug("in plugui_onload");
}


/*
function cb(result) {
  dialogMessage("Result", "Callback result: " + result.tasks,
  {
   buttons:
   {
     Ok: function() {
       jQuery(this).dialog('close');
     }
   }
  });
}
*/
