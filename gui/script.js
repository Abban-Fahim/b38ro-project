/**
 * @
 */
const ros = new ROSLIB.Ros();

ros.connect("ws://localhost:9090");


const bruhTopic = new ROSLIB.Topic({
    ros, name: "/text", messageType: "std_msgs/String"
});

bruhTopic.subscribe((msg)=>{
    console.log(msg.data);
})
