const express = require('express');
const path = require('path');
const app = express();
const PORT = 8080;

// Since server.js is INSIDE the web folder, 
// __dirname is already pointing to the web directory.
app.use(express.static(__dirname));

// Explicitly serve index.html for the root route
app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, 'index.html'));
});

app.listen(PORT, '0.0.0.0', () => {
    console.log(`Web UI is running!`);
    console.log(`URL: http://localhost:${PORT}`);
    console.log(`Serving files from: ${__dirname}`);
});