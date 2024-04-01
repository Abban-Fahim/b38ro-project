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

let global_joint_angles = [];
joint_angles.subscribe((msg)=>{
   global_joint_angles[0] = msg.position[0];
   for (let i = 2; i < 7; i++) {
    global_joint_angles[i-1] = msg.position[i];
   }
});

let global_end_effector_transform = {pos: {}, angles: {}};
robot_position.subscribe((msg)=>{
    global_end_effector_transform.pos = msg.position;
    ({x, y, z, w} = msg.orientation);
    // let phi = Math.atan2(2*(x*w + y*z), 1 - 2*(x^2 + y^2))
    // let theta = Math.atan2(2*(x*w + y*z), 1 - 2*(x^2 + y^2))
    // let psi = Math.atan2(2*(x*w + y*z), 1 - 2*(x^2+y^2))
    
    // Algorithm for calculating extrinsic XYZ Euler angles
    
    const i = 0, j = 1, k = 2;
    const sign = Math.floor((i-j)*(j-k)*(k-i)/2);
    
    const a = w - y, b = x + z * sign, c = y + w, d = z*sign - x;
    const n2 = a^2 + b^2 + c^2 + d^2;

    let theta1, theta3;
    let theta2 = Math.acos(2*(a^2 + b^2) / (n2 - 1))
    console.log(theta2)
    
    const eps = 0.00000001;
    const safe1 = Math.abs(theta2) >= eps;
    const safe2 = Math.abs(theta2 - Math.PI) >= eps;
    const safe = safe1 && safe2;

    if (safe) {
        const half_sum = Math.atan2(b, a);
        const half_diff = Math.atan2(-d, c);

        theta1 = half_sum + half_diff;
        theta3 = half_sum - half_diff;

    } else {
        theta3 = 0;
        
        if (!safe1) {
            const half_sum = Math.atan2(b, a);
            theta1 = 2*half_sum;
        } 
        if (!safe2) {
            const half_diff = Math.atan2(-d, c);
            theta1 = 2*half_diff;
        }
    }

    theta3 *= sign;
    theta2 -= Math.PI / 2;

    // console.log(theta1, theta2, theta3);
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
    cartesian_pos.publish({position: {x, y, z}})
}

const angles_chart = new Chart(document.getElementById("joint_angles"), {
    type: "line",
    data: {
        datasets: [
            {
                label: "Joint 0",
                borderColor: "#BF616A",
                data: []
            },
            {
                label: "Joint 1",
                borderColor: "#D08770",
                data: []
            },
            {
                label: "Joint 2",
                borderColor: "#EBCB8B",
                data: []
            },
            {
                label: "Joint 3",
                borderColor: "#A3BE8C",
                data: []
            },
            {
                label: "Joint 4",
                borderColor: "#88C0D0",
                data: []
            },
            {
                label: "Joint 5",
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
                                y: global_joint_angles[i]
                            })
                        })
                    }
                }
            }
        }
    }
})

// const modelViewer = new ROS3D.Viewer({
//     divID: "urdf",
//     width: 800,
//     height: 600,
//     antialias: true
// })

// const tfClient = new ROSLIB.TFClient({
//     ros,
//     angularThres: 0.01,
//     transThres: 0.01,
//     rate: 10.0
// });

// const urdfClient = new ROS3D.UrdfClient({
//     ros,
//     tfClient: tfClient,
//     path: 'http://resources.robotwebtools.org/',
//     rootObject : modelViewer.scene,
//     loader: ROS3D.COLLADA_LOADER_2
// });

// modelViewer.addObject(new ROS3D.Grid());

