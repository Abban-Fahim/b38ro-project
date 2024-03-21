/**
 * @
 */
const ros = new ROSLIB.Ros();

ros.connect("ws://10.6.156.130:9090");

const cartesian_pos = new ROSLIB.Topic({
    ros, name: "/cart_pose", messageType: "std_msgs/Float32MultiArray"
});

const imgTopic = new ROSLIB.Topic({
    ros, name: "/camera/image/compressed", messageType: 'sensor_msgs/CompressedImage'
});

const canvas = document.querySelector("canvas");
const video = document.querySelector("video");

function takePic() {
    navigator.mediaDevices.getUserMedia({video: true, audio: false}).then((stream)=>{
        video.srcObject = stream;
        canvas.getContext("2d").drawImage(video, 0, 0);
        const data = canvas.toDataURL("image/jpeg");
        imgTopic.publish({format: "jpeg", data: data.replace("data:image/jpeg;base64,", "")})
    });
}

document.getElementById("start-vid").onclick = ()=>{
    console.log("started video");
    setInterval(takePic, 100);
};

function capture() {
    canvas.getContext("2d").drawImage(video, 0, 0)
}

// bruhTopic.subscribe((msg)=>{
//     console.log(msg.data);
// })

pos_x = document.getElementById("pos_x");
pos_y = document.getElementById("pos_y");
pos_z = document.getElementById("pos_z");

[pos_x, pos_y, pos_z].forEach((el)=>{
    el.onchange = (e) => {
        document.getElementById(el.id.replace("pos","label")).innerHTML = el.value;
    }
})

document.getElementById("move").onclick = (e) => {
    let x = Number(pos_x.value);
    let y = Number(pos_y.value);
    let z = Number(pos_z.value);
    console.log(x,y,z);
    cartesian_pos.publish({data: [x, y, z]})
}

cartesian_pos.publish({data: [0.2, 0.3, 0.4]})