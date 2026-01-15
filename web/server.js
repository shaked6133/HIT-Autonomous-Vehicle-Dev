const express = require('express');
const path = require('path');
const app = express();
const PORT = 8080;

// 1. Serve ALL static files (html, js, images, icons) from the /web folder
app.use(express.static(path.join(__dirname, 'web')));

// 2. Explicitly route the root URL to /web/index.html
app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, 'web', 'index.html'));
});

app.listen(PORT, '0.0.0.0', () => {
    console.log(`Web UI is running at http://localhost:${PORT}`);
});