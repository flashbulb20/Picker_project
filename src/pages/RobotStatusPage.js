import React from "react";

function RobotStatusPage() {

  const sendDock = async () => {
    await fetch("http://192.168.107.61:8000/robot/dock", {
      method: "POST"
    });
    alert("Dock 명령 전송됨");
  };

  const sendUndock = async () => {
    await fetch("http://192.168.107.61:8000/robot/undock", {
      method: "POST"
    });
    alert("Undock 명령 전송됨");
  };

  return (
    <div>
      <h2>로봇 상태 페이지</h2>

      <button onClick={sendDock}>Dock</button>
      <button onClick={sendUndock}>Undock</button>
    </div>
  );
}

export default RobotStatusPage;
