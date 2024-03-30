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
    ros, name: "/robot_state", messageType: "std_msgs/Float32MultiArray"
});

let global_joint_angles = 0;
joint_angles.subscribe((msg)=>{
   global_joint_angles = msg.data;
});

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