  import React, { useEffect, useState } from "react";
  import { useNavigate } from "react-router-dom";

  const API_BASE = "http://192.168.107.61:8000"; // 로봇 서버 주소

  function OrderPage() {
    const navigate = useNavigate();

    // 상태 관리
    const [boxes, setBoxes] = useState([]);
    const [loading, setLoading] = useState(true);
    const [error, setError] = useState("");
    
    // 선택된 값들
    const [selectedSize, setSelectedSize] = useState("");
    const [selectedRobot, setSelectedRobot] = useState("robot3"); // 기본값 robot3
    const [selectedDest, setSelectedDest] = useState("ChangingRoom1"); // 기본값 ChangingRoom1
    
    const [orderMessage, setOrderMessage] = useState("");

    // 페이지 들어오면 박스 재고 조회
    useEffect(() => {
      const fetchBoxes = async () => {
        setLoading(true);
        setError("");
        try {
          const res = await fetch(`${API_BASE}/boxes`);
          const data = await res.json();

          if (!res.ok) {
            const reason = data?.detail || data?.error || `HTTP ${res.status}`;
            setError(`재고 조회 실패: ${reason}`);
          } else {
            // data가 배열인지 확인
            if (Array.isArray(data)) {
              setBoxes(data);
            } else {
              setBoxes([]);
              setError("서버 응답 형식이 올바르지 않습니다.");
            }
          }
        } catch (err) {
          setError(`서버 오류: ${err.message}`);
        } finally {
          setLoading(false);
        }
      };

      fetchBoxes();
    }, []);

    // 주문 버튼 클릭
    const handleOrder = async () => {
      if (!selectedSize) {
        alert("주문할 박스 사이즈를 선택해주세요.");
        return;
      }

      setOrderMessage("⏳ 주문 처리 중...");
      
      try {
        // API 요청에 robot_id와 destination 추가
        const res = await fetch(`${API_BASE}/orders`, {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
          },
          body: JSON.stringify({ 
            size: selectedSize,
            robot_id: selectedRobot,
            destination: selectedDest
          }),
        });

        const data = await res.json();

        if (!res.ok) {
          const reason = data?.detail || data?.error || `HTTP ${res.status}`;
          setOrderMessage(`❌ 주문 실패: ${reason}`);
          return;
        }

        setOrderMessage(
          `✅ 주문 성공! [${selectedRobot}호]가 [${selectedDest}]로 출발합니다. (남은 재고: ${data.remaining_quantity})`
        );

        // 재고 화면 즉시 갱신
        setBoxes((prev) =>
          prev.map((b) =>
            b.size === selectedSize
              ? { ...b, quantity: data.remaining_quantity }
              : b
          )
        );
      } catch (err) {
        setOrderMessage(`❌ 서버 오류: ${err.message}`);
      }
    };

    return (
      <>
        <style>{`
          .order-container {
            max-width: 600px;
            margin: 0 auto;
            padding: 20px;
            font-family: 'Noto Sans KR', sans-serif;
          }
          h1 {
            text-align: center;
            color: #333;
          }
          p {
            text-align: center;
            color: #666;
          }
          .back-btn {
            display: block;
            margin: 0 auto 20px;
            background: none;
            border: 1px solid #ccc;
            padding: 8px 16px;
            cursor: pointer;
            border-radius: 4px;
          }
          .back-btn:hover {
            background-color: #f0f0f0;
          }
          .section-title {
            font-size: 1.1rem;
            font-weight: bold;
            margin: 20px 0 10px;
            color: #444;
            border-bottom: 2px solid #eee;
            padding-bottom: 5px;
          }
          .box-list {
            display: grid;
            grid-template-columns: repeat(auto-fill, minmax(100px, 1fr));
            gap: 15px;
            margin-bottom: 20px;
          }
          .box-item {
            border: 2px solid #ddd;
            border-radius: 12px;
            padding: 15px;
            text-align: center;
            cursor: pointer;
            transition: all 0.2s;
            background: white;
          }
          .box-item:hover {
            transform: translateY(-2px);
            box-shadow: 0 4px 8px rgba(0,0,0,0.1);
          }
          .box-item.selected {
            border-color: #007bff;
            background-color: #e7f1ff;
            color: #007bff;
          }
          .box-item.disabled {
            opacity: 0.5;
            cursor: not-allowed;
            background-color: #f9f9f9;
          }
          .soldout {
            display: block;
            color: red;
            font-size: 0.8rem;
            font-weight: bold;
            margin-top: 5px;
          }
          .options-container {
            background-color: #f8f9fa;
            padding: 20px;
            border-radius: 12px;
            margin-bottom: 25px;
          }
          .option-group {
            margin-bottom: 15px;
          }
          .option-group:last-child {
            margin-bottom: 0;
          }
          .option-group label {
            display: block;
            margin-bottom: 8px;
            font-weight: bold;
            color: #555;
          }
          .custom-select {
            width: 100%;
            padding: 12px;
            border: 1px solid #ddd;
            border-radius: 8px;
            font-size: 1rem;
            background-color: white;
          }
          .order-btn {
            width: 100%;
            padding: 16px;
            background-color: #007bff;
            color: white;
            border: none;
            border-radius: 12px;
            font-size: 1.2rem;
            font-weight: bold;
            cursor: pointer;
            transition: background-color 0.2s;
          }
          .order-btn:hover:not(:disabled) {
            background-color: #0056b3;
          }
          .order-btn:disabled {
            background-color: #ccc;
            cursor: not-allowed;
          }
          .order-message {
            margin-top: 20px;
            padding: 15px;
            background-color: #e8f5e9;
            border-radius: 8px;
            text-align: center;
            font-weight: bold;
            color: #2e7d32;
            white-space: pre-line;
          }
          .error-text {
            color: #d32f2f;
            text-align: center;
            margin-top: 20px;
          }
        `}</style>

        <div className="order-container">
          <h1>📦 상품 주문 및 배달 요청</h1>
          <p>원하는 박스와 배달할 로봇/위치를 선택하세요.</p>

          <button className="back-btn" onClick={() => navigate("/main")}>
            ⬅ 메인으로 돌아가기
          </button>

          {loading && <p>재고 정보를 불러오는 중입니다...</p>}
          {error && !loading && <p className="error-text">{error}</p>}

          {!loading && !error && (
            <>
              {/* 1. 박스 선택 영역 */}
              <div className="section-title">1. 박스 사이즈 선택</div>
              <div className="box-list">box-list
                {boxes.map((box) => (
                  <div
                    key={box.size}
                    className={`box-item ${
                      box.quantity <= 0 ? "disabled" : ""
                    } ${selectedSize === box.size ? "selected" : ""}`}
                    onClick={() => {
                      if (box.quantity > 0) {
                        setSelectedSize(box.size);
                      }
                    }}
                  >
                    <h3>{box.size}</h3>
                    <p>재고: {box.quantity}</p>
                    {box.quantity <= 0 && <span className="soldout">품절</span>}
                  </div>
                ))}
              </div>

              {/* 2. 배달 옵션 선택 영역 */}
              <div className="section-title">2. 배달 옵션 선택</div>
              <div className="options-container">
                <div className="option-group">
                  <label>🤖 배달 로봇 선택:</label>
                  <select 
                    value={selectedRobot} 
                    onChange={(e) => setSelectedRobot(e.target.value)}
                    className="custom-select"
                  >
                    <option value="robot3">로봇 1호 (Robot 3)</option>
                    <option value="robot2">로봇 2호 (Robot 2)</option>
                  </select>
                </div>

                <div className="option-group">
                  <label>🏁 목적지(탈의실) 선택:</label>
                  <select 
                    value={selectedDest} 
                    onChange={(e) => setSelectedDest(e.target.value)}
                    className="custom-select"
                  >
                    <option value="ChangingRoom1">탈의실 1번 (ChangingRoom1)</option>
                    <option value="ChangingRoom2">탈의실 2번 (ChangingRoom2)</option>
                  </select>
                </div>
              </div>

              <button
                className="order-btn"
                onClick={handleOrder}
                disabled={!selectedSize}
              >
                {selectedSize
                  ? `[${selectedSize}] 주문 및 배달 시작`
                  : "박스 사이즈를 선택해주세요"}
              </button>

              {orderMessage && <div className="order-message">{orderMessage}</div>}
            </>
          )}
        </div>
      </>
    );
  }

  export default OrderPage;