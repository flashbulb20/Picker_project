import React, { useState, useEffect, useCallback } from "react";
import { useNavigate } from "react-router-dom";
import "./MainPage.css";

// FastAPI 서버 주소
const API_BASE = "http://192.168.107.61:8000";

function MainPage() {
  const navigate = useNavigate();

  // ✅ [화면 모드 관리] 'menu' | 'robot' | 'cctv'
  const [viewMode, setViewMode] = useState("menu");

  // --- 로봇 제어 관련 상태 ---
  const [activeRobot, setActiveRobot] = useState("robot3");
  const [showRobotCamera, setShowRobotCamera] = useState(false); // 로봇 카메라 on/off
  
  // 로봇 상태 데이터
  const [robotStatus, setRobotStatus] = useState(null);
  const [statusLoading, setStatusLoading] = useState(false);
  const [statusError, setStatusError] = useState("");

  // ✅ 로봇이 바뀌면 상태 초기화
  useEffect(() => {
    setRobotStatus(null);
    setStatusError("");
  }, [activeRobot]);

  // ✅ 상태 조회 함수 (useCallback)
  const fetchRobotStatus = useCallback(async (isAuto = false) => {
    // 로봇 제어 화면일 때만 실행
    if (viewMode !== "robot") return;

    if (!isAuto) setStatusLoading(true);
    if (!isAuto) setStatusError("");

    try {
      const res = await fetch(`${API_BASE}/robot/${activeRobot}/status`);
      const data = await res.json();

      if (!res.ok) {
        if (!isAuto) {
          const reason = data?.detail || data?.error || `HTTP ${res.status}`;
          setStatusError(reason);
        }
        return;
      }
      setRobotStatus(data);
    } catch (err) {
      if (!isAuto) {
        setStatusError(err.message || "서버 통신 오류");
      }
    } finally {
      if (!isAuto) setStatusLoading(false)
    }
  }, [activeRobot, viewMode]);

  // ✅ 1초마다 상태 자동 갱신 (로봇 제어 모드일 때만)
  useEffect(() => {
    if (viewMode === "robot") {
      fetchRobotStatus(true);
      const intervalId = setInterval(() => {
        fetchRobotStatus(true);
      }, 5000);
      return () => clearInterval(intervalId);
    }
  }, [fetchRobotStatus, viewMode]);


  // --- 명령 전송 함수들 ---
  const sendCmdVel = async (linear_x, angular_z) => {
    try {
      await fetch(`${API_BASE}/robot/${activeRobot}/cmd_vel`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ linear_x, angular_z }),
      });
    } catch (err) {
      console.error(err);
    }
  };

  const sendDock = async () => {
    try {
      await fetch(`${API_BASE}/robot/${activeRobot}/dock`, { method: "POST" });
      alert(`[${activeRobot}] 충전 복귀 명령 전송!`);
    } catch (err) {
      alert(`오류: ${err.message}`);
    }
  };

  const sendUndock = async () => {
    try {
      await fetch(`${API_BASE}/robot/${activeRobot}/undock`, { method: "POST" });
      alert(`[${activeRobot}] 도킹 해제 명령 전송!`);
    } catch (err) {
      alert(`오류: ${err.message}`);
    }
  };

  // --- UI 텍스트 포맷팅 ---
  const batteryText = robotStatus?.battery_percentage != null 
    ? `${robotStatus.battery_percentage.toFixed(1)} %` : "정보 없음";
  
  const dockText = robotStatus?.docked != null 
    ? (robotStatus.docked ? "충전 중 (Docked)" : "이동 중 (Undocked)") : "확인 중...";


  // =================================================================
  // 1️⃣ 메인 메뉴 화면 (버튼 3개)
  // =================================================================
  const renderMenu = () => (
    <div className="menu-container" style={{ marginTop: "50px", display: 'flex', flexDirection: 'column', gap: '20px', alignItems: 'center' }}>
      <button 
        className="main-menu-btn order"
        onClick={() => navigate("/order")}
        style={{ padding: "30px", fontSize: "24px", width: "80%", maxWidth: "400px", borderRadius: "15px", backgroundColor: "#ffc107", border: "none", cursor: "pointer", color: "#000", fontWeight: "bold", boxShadow: "0 4px 6px rgba(0,0,0,0.2)" }}
      >
        📦 주문하기
      </button>

      <button 
        className="main-menu-btn robot"
        onClick={() => setViewMode("robot")}
        style={{ padding: "30px", fontSize: "24px", width: "80%", maxWidth: "400px", borderRadius: "15px", backgroundColor: "#007bff", border: "none", cursor: "pointer", color: "#fff", fontWeight: "bold", boxShadow: "0 4px 6px rgba(0,0,0,0.2)" }}
      >
        🤖 로봇 제어
      </button>

      <button 
        className="main-menu-btn cctv"
        onClick={() => setViewMode("cctv")}
        style={{ padding: "30px", fontSize: "24px", width: "80%", maxWidth: "400px", borderRadius: "15px", backgroundColor: "#dc3545", border: "none", cursor: "pointer", color: "#fff", fontWeight: "bold", boxShadow: "0 4px 6px rgba(0,0,0,0.2)" }}
      >
        📹 CCTV 보기
      </button>
    </div>
  );

  // =================================================================
  // 2️⃣ 로봇 제어 화면
  // =================================================================
  const renderRobotControl = () => (
    <div className="robot-mode-container">
      <button className="back-btn" onClick={() => setViewMode("menu")} style={{ marginBottom: "20px", padding: "10px 20px", backgroundColor: "#6c757d", color: "white", border: "none", borderRadius: "5px", cursor: "pointer" }}>
        ⬅ 메인으로 돌아가기
      </button>

      {/* 로봇 선택 */}
      <div style={{ marginBottom: "20px", textAlign: "center" }}>
        {["robot2", "robot3"].map((id) => (
          <button
            key={id}
            onClick={() => setActiveRobot(id)}
            style={{
              backgroundColor: activeRobot === id ? "#007bff" : "#333",
              color: "#fff",
              margin: "0 10px",
              padding: "15px 30px",
              border: activeRobot === id ? "3px solid #fff" : "none",
              borderRadius: "10px",
              cursor: "pointer",
              fontWeight: "bold",
              fontSize: "18px"
            }}
          >
            {id.toUpperCase()}
          </button>
        ))}
      </div>

      {/* 로봇 카메라 토글 */}
      <div style={{textAlign: "center", marginBottom: "20px"}}>
        <button 
          onClick={() => setShowRobotCamera(!showRobotCamera)}
          style={{ padding: "10px 20px", borderRadius: "20px", border: "none", backgroundColor: showRobotCamera ? "#28a745" : "#6c757d", color: "white", cursor: "pointer", fontWeight: "bold" }}
        >
          {showRobotCamera ? "📺 로봇 카메라 끄기" : "📺 로봇 카메라 켜기"}
        </button>
      </div>

      {/* 로봇 카메라 화면 */}
      {showRobotCamera && (
        <div className="camera-box" style={{ textAlign: "center", marginBottom: "20px" }}>
          <div style={{ width: "640px", height: "480px", backgroundColor: "#000", margin: "0 auto", border: "5px solid #444", borderRadius: "10px", overflow: "hidden", display: "flex", alignItems: "center", justifyContent: "center" }}>
            <img 
              src={`${API_BASE}/robot/${activeRobot}/camera`} 
              alt="Robot Camera"
              style={{ width: "100%", height: "100%", objectFit: "contain" }}
              onError={(e) => { e.target.style.display='none'; e.target.parentNode.innerText = "🔌 카메라 신호 없음"; }}
            />
          </div>
        </div>
      )}

      {/* 상태 정보 */}
      <div className="robot-status-box" style={{ padding: "20px", border: "2px solid #007bff", borderRadius: "10px", marginBottom: "20px", backgroundColor: "#1e1e1e" }}>
        <h3>📡 {activeRobot.toUpperCase()} 상태</h3>
        {!statusLoading && !statusError && (
          <div style={{ fontSize: "1.3rem", display: "flex", justifyContent: "space-around" }}>
            <span>🔋 {batteryText}</span>
            <span>⚓ {dockText}</span>
          </div>
        )}
        {statusLoading && <p>데이터 수신 중...</p>}
      </div>

      {/* 제어 버튼 (Dock/Undock) */}
      <div className="robot-control-box" style={{ textAlign: "center", marginBottom: "20px" }}>
        <button onClick={sendDock} style={{ padding: "15px 30px", marginRight: "10px", fontSize: "18px", backgroundColor: "#17a2b8", color: "white", border: "none", borderRadius: "8px", cursor: "pointer" }}>🏠 복귀 (Dock)</button>
        <button onClick={sendUndock} style={{ padding: "15px 30px", fontSize: "18px", backgroundColor: "#fd7e14", color: "white", border: "none", borderRadius: "8px", cursor: "pointer" }}>🚀 출동 (Undock)</button>
      </div>

      {/* 조이스틱 */}
      <div className="teleop-box">
        <h3 style={{textAlign:"center"}}>🕹 조이스틱</h3>
        <div className="teleop-grid">
          <button className="teleop-btn" onClick={() => sendCmdVel(0.2, 0.0)}>▲</button>
          <div className="teleop-middle-row">
            <button className="teleop-btn" onClick={() => sendCmdVel(0.0, 0.5)}>◀</button>
            <button className="teleop-btn stop" onClick={() => sendCmdVel(0.0, 0.0)}>⏹</button>
            <button className="teleop-btn" onClick={() => sendCmdVel(0.0, -0.5)}>▶</button>
          </div>
          <button className="teleop-btn" onClick={() => sendCmdVel(-0.2, 0.0)}>▼</button>
        </div>
      </div>
    </div>
  );

  // =================================================================
  // 3️⃣ CCTV 화면
  // =================================================================
  const renderCCTV = () => (
    <div className="cctv-mode-container" style={{ textAlign: "center" }}>
      <button className="back-btn" onClick={() => setViewMode("menu")} style={{ marginBottom: "20px", padding: "10px 20px", backgroundColor: "#6c757d", color: "white", border: "none", borderRadius: "5px", cursor: "pointer" }}>
        ⬅ 메인으로 돌아가기
      </button>

      <h2 style={{color: "#dc3545"}}>🚨 실시간 보안 관제 (CCTV)</h2>
      
      <div className="cctv-box" style={{ marginTop: "20px" }}>
        <div style={{ 
            width: "800px", 
            height: "600px", 
            backgroundColor: "#000", 
            margin: "0 auto", 
            border: "10px solid #333", 
            borderRadius: "15px", 
            overflow: "hidden",
            display: "flex", 
            alignItems: "center", 
            justifyContent: "center",
            position: "relative"
        }}>
          <img 
            src={`${API_BASE}/cctv/stream`} 
            alt="CCTV Stream"
            style={{ width: "100%", height: "100%", objectFit: "contain" }}
            onError={(e) => {
               e.target.style.display='none'; 
               e.target.parentNode.style.color = "#ffc107";
               e.target.parentNode.innerText = "🔌 CCTV 신호 없음 (서버 확인 필요)";
            }}
          />
          {/* 녹화 중 표시 (데코레이션) */}
          <div style={{ position: "absolute", top: "20px", right: "20px", color: "red", fontWeight: "bold", fontSize: "20px", animation: "blink 1s infinite" }}>
            ● REC
          </div>
        </div>
        <p style={{ marginTop: "15px", color: "#aaa" }}>* ROI 영역 감지 중</p>
      </div>
    </div>
  );

  // =================================================================
  // 실제 렌더링 부분
  // =================================================================
  return (
    <div className="main-container">
      <h1 style={{ marginBottom: "10px" }}>ROKEY 통합 관제 시스템</h1>
      <p style={{ color: "#aaa", marginBottom: "30px" }}>원하는 기능을 선택해주세요.</p>

      {viewMode === "menu" && renderMenu()}
      {viewMode === "robot" && renderRobotControl()}
      {viewMode === "cctv" && renderCCTV()}
    </div>
  );
}

export default MainPage;