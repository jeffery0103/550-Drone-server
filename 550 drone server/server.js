// 1. 引入必要的套件 (記得 npm install express ws)
const express = require('express');
const { WebSocketServer } = require('ws');
const http = require('http');
const path = require('path');

// 2. 建立 Express 網頁伺服器
const app = express();
app.use(express.static(path.join(__dirname, 'public')));
const server = http.createServer(app);

// 3. 建立 WebSocket 伺服器 (附著在同一個 port 上)
const wss = new WebSocketServer({ server });

function broadcastTelemetry(dataStr) {
    wss.clients.forEach(function each(client) {
        if (client.readyState === 1 /* OPEN */) {
            client.send(dataStr);
        }
    });
}

// 4. 當有人連上 WebSocket 時 (可能是 ESP32，也可能是你的網頁)
wss.on('connection', (ws, req) => {
    console.log('有新裝置連線了！來源:', req.socket.remoteAddress);

    // 當收到訊息時
    ws.on('message', (message) => {
        const dataStr = message.toString();
        // console.log('收到飛機數據:', dataStr); // 測試時可以打開這行

        // 🚀 核心邏輯：把收到的訊息，無腦廣播給「所有其他」正在連線的裝置（你的網頁）
        wss.clients.forEach(function each(client) {
            if (client !== ws && client.readyState === 1 /* OPEN */) {
                client.send(dataStr);
            }
        });
    });

    ws.on('close', () => {
        console.log('裝置斷線');
    });
});


// Render 會提供 process.env.PORT，沒有的話預設用 3000
const PORT = process.env.PORT || 3000;
server.listen(PORT, () => {
    console.log(`伺服器已經在 Port ${PORT} 啟動啦！`);
});