// import * as THREE from "three";
const ros = new ROSLIB.Ros();

ros.connect("ws://localhost:9090");

Chart.defaults.set('plugins.streaming', {
    duration: 20000
});
Chart.defaults.color = "#D8DEE9"
Chart.defaults.font.family = 'Ubuntu Mono'
Chart.defaults.font.size = 16

const cartesian_pos = new ROSLIB.Topic({
    ros, name: "/cart_pose", messageType: "geometry_msgs/Pose"
});
const joint_angles = new ROSLIB.Topic({
    ros, name: "/joint_states", messageType: "sensor_msgs/JointState"
});
const robot_position = new ROSLIB.Topic({
    ros, name: "/robot_position", messageType: "geometry_msgs/Pose"
});
const robot_jacobian = new ROSLIB.Topic({
    ros, name: "/robot_jacobian", messageType: "std_msgs/Float32MultiArray"
});

let global_joint_angles = [0, 0, 0, 0, 0, 0];
let global_joint_velocities = [0, 0, 0, 0, 0, 0];
joint_angles.subscribe((msg)=>{
   for (let i = 0; i < msg.name.length; i++) {
    if (msg.name[i].startsWith("joint_")) {
        let index = Number(msg.name[i][msg.name[i].length - 1]) - 1;
        global_joint_angles[index] = msg.position[i];
        global_joint_velocities[index] = msg.velocity[i];
    }
   }
});

// Stores current end effector transformation of arm
let eef_tf = {pos: {}, angles: {}};
const arm_pos_display = document.getElementById("arm_pos");
const arm_rot_display = document.getElementById("arm_rot");
robot_position.subscribe((msg)=>{
    eef_tf.pos = msg.position;
    let x, y, z;
    // Recive ZYX euler angles of the gripper's orientation
    ({z, y, x} = msg.orientation);
    eef_tf.angles = {x, y, z};
    arm_pos_display.innerHTML = `Cartesian position: X: ${eef_tf.pos.x}, Y: ${eef_tf.pos.y}, Z: ${eef_tf.pos.z}`;
    arm_rot_display.innerHTML = `Euler angles: Z: ${eef_tf.angles.z}, Y: ${eef_tf.pos.y}, X: ${eef_tf.angles.x}`;
    
})

let jacobian = [];
const JTable = document.querySelector("table");
robot_jacobian.subscribe((msg)=>{
    // Clear table
    jacobian = [];
    JTable.childNodes[3].innerHTML = "";

    for (let i = 0; i < 6; i++) {
        // Clear row
        let row = [];
        
        // Create next row
        let tr = document.createElement("tr");
        for (let j = 0; j < 6; j++) {
            row.push(msg.data[i*6 + j]);
            let td = document.createElement("td");
            td.innerText = msg.data[i*6 + j];
            tr.appendChild(td);
        }
   
        // Append row to table body
        JTable.childNodes[3].appendChild(tr);
        jacobian.push(row);
    }
})

pos_x = document.getElementById("pos_x");
pos_y = document.getElementById("pos_y");
pos_z = document.getElementById("pos_z");

[pos_x, pos_y, pos_z].forEach((el)=>{
    el.onchange = ()=>{updateLabels(el)};
    updateLabels(el);
})
function updateLabels(el) {
    document.getElementById(el.id.replace("pos","label")).innerHTML = el.value;
}


document.getElementById("move").onclick = (e) => {
    let x = Number(pos_x.value);
    let y = Number(pos_y.value);
    let z = Number(pos_z.value);
    // Send input cartesian coordinates and euler angles denoting pointing down
    cartesian_pos.publish({position: {x, y, z}, orientation: {x:Math.PI, y:0, z:0}})
}

const new_chart = (array, elID, axesLabel) => new Chart(document.getElementById(elID), {
    type: "line",
    data: {
        datasets: [
            {
                label: "Joint 1",
                borderColor: "#BF616A",
                data: []
            },
            {
                label: "Joint 2",
                borderColor: "#D08770",
                data: []
            },
            {
                label: "Joint 3",
                borderColor: "#EBCB8B",
                data: []
            },
            {
                label: "Joint 4",
                borderColor: "#A3BE8C",
                data: []
            },
            {
                label: "Joint 5",
                borderColor: "#88C0D0",
                data: []
            },
            {
                label: "Joint 6",
                borderColor: "#B48EAD",
                data: []
            },

        ]
    },
    options: {
        scales: {
            x: {
                type: "realtime",
                realtime: {
                    onRefresh: chart => {
                        chart.data.datasets.forEach((dataset, i)=>{
                            dataset.data.push({
                                x: Date.now(),
                                y: array[i]
                            })
                        })
                    }
                }
            },
            y: {
                title: {
                    text: axesLabel,
                    display: true
                },
            }
        }
    }
})

const angles_chart = new_chart(global_joint_angles, "joint_angles", "(rad)");
const vel_chart = new_chart(global_joint_velocities, "joint_vel", "rad/s");