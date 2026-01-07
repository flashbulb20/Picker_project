import React, { useState } from "react";
import { useNavigate } from "react-router-dom";
import "./LoginPage.css";

function LoginPage() {
  const [userId, setUserId] = useState("");
  const [password, setPassword] = useState("");
  const [message, setMessage] = useState("");

  const navigate = useNavigate();

  const handleLogin = async () => {
    console.log("✅ [1] 로그인 버튼 클릭됨");
    console.log("👉 입력된 ID:", userId);
    console.log("👉 입력된 PW:", password);

    try {
      console.log("✅ [2] fetch 요청 시작");

        const response = await fetch("http://192.168.107.61:8000/login", {
        method: "POST",
        headers: {
          "Content-Type": "application/json",
        },
        body: JSON.stringify({
          user_id: userId,
          password: password,
        }),
      });

      console.log("✅ [3] 서버 응답 도착");
      console.log("👉 status:", response.status);
      console.log("👉 ok:", response.ok);

      const text = await response.text();
      console.log("✅ [4] raw response text:", text);

      let data = null;
      try {
        data = JSON.parse(text);
        console.log("✅ [5] JSON 변환 성공:", data);
      } catch (e) {
        console.error("❌ JSON 파싱 실패:", e);
      }

      if (response.ok) {
        console.log("✅ [6] 로그인 성공 처리");
        setMessage("로그인 성공!");
        navigate("/main");
      } else {
        console.log("❌ [6] 로그인 실패 처리");

        if (data && data.detail) {
          const msg = Array.isArray(data.detail)
            ? data.detail[0].msg
            : data.detail;
          setMessage(msg);
        } else {
          setMessage("로그인 실패");
        }
      }
    } catch (error) {
      console.error("❌ [X] fetch 자체 실패:", error);
      setMessage("서버 오류 발생 (네트워크)");
    }
  };

  return (
    <div className="login-container">
      <div className="login-form">
        <h2>로그인</h2>

        <input
          type="text"
          placeholder="User ID"
          value={userId}
          onChange={(e) => setUserId(e.target.value)}
        />

        <input
          type="password"
          placeholder="Password"
          value={password}
          onChange={(e) => setPassword(e.target.value)}
        />

        <button onClick={handleLogin}>로그인</button>

        <p className="message">{message}</p>
      </div>
    </div>
  );
}

export default LoginPage;
