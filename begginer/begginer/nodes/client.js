
// Main variables for logs and peer connection
var dataChannelLog = document.getElementById('data-channel'),
    iceConnectionLog = document.getElementById('ice-connection-state'),
    iceGatheringLog = document.getElementById('ice-gathering-state'),
    signalingLog = document.getElementById('signaling-state');

var pc = null; // Variable for the PeerConnection
var dc = null, dcInterval = null; // DataChannel and its interval

// Function to create a new peer connection
function createPeerConnection() {
    var config = {
        sdpSemantics: 'unified-plan'
    };


    pc = new RTCPeerConnection(config); // Initialize the PeerConnection

    // Event listeners for various peer connection states
    pc.addEventListener('icegatheringstatechange', function() {
        iceGatheringLog.textContent += ' -> ' + pc.iceGatheringState;
        console.log('ICE Gathering State changed:', pc.iceGatheringState); // Log for debugging
    }, false);
    iceGatheringLog.textContent = pc.iceGatheringState;

    pc.addEventListener('iceconnectionstatechange', function() {
        iceConnectionLog.textContent += ' -> ' + pc.iceConnectionState;
        console.log('ICE Connection State changed:', pc.iceConnectionState); // Log for debugging
    }, false);
    iceConnectionLog.textContent = pc.iceConnectionState;

    pc.addEventListener('signalingstatechange', function() {
        signalingLog.textContent += ' -> ' + pc.signalingState;
        console.log('Signaling State changed:', pc.signalingState); // Log for debugging
    }, false);
    signalingLog.textContent = pc.signalingState;

    pc.addEventListener('track', function(evt) {
        console.log('Track event:', evt); // Log for debugging
        if (evt.track.kind == 'video')
            document.getElementById('video').srcObject = evt.streams[0];
        else
            document.getElementById('audio').srcObject = evt.streams[0];
    });

    return pc;
}

// Function to negotiate the peer connection
function negotiate()

{
    var videoResolutionSelect = document.getElementById('video-resolution');
    var selectedResolution = videoResolutionSelect.value;
    console.log('Selected video resolution:', selectedResolution); // Log for debugging

    // Create offer and set local description
    return pc.createOffer({offerToReceiveVideo: true}).then(function(offer) {
        return pc.setLocalDescription(offer);
    }).then(function() {
        return new Promise(function(resolve) {
            if (pc.iceGatheringState === 'complete') {
                resolve();
            } else {
                function checkState() {
                    if (pc.iceGatheringState === 'complete') {
                        pc.removeEventListener('icegatheringstatechange', checkState);
                        resolve();
                    }
                }
                pc.addEventListener('icegatheringstatechange', checkState);
            }
        });
    }).then(function() {
        console.log('Sending offer'); // Log for debugging
        // Send the offer to the server
        return fetch('/offer', {
            body: JSON.stringify({
                sdp: pc.localDescription.sdp,
                type: pc.localDescription.type,
                video_resolution: selectedResolution
            }),
            headers: {
                'Content-Type': 'application/json'
            },
            method: 'POST'
        });

    }).then(function(response) {
        return response.json();
    }).then(function(answer) { // Log for debugging
        return pc.setRemoteDescription(answer);
    }).catch(function(e) { // Log for error
        alert(e);
    });
}

// Function to start the peer connection and data channel
function start() {
    document.getElementById('start').style.display = 'none';
    pc = createPeerConnection();
    console.log('Peer connection created'); // Log for debugging
    var time_start = null;

    function current_stamp() {
        if (time_start === null) {
            time_start = new Date().getTime();
            return 0;
        } else {
            return new Date().getTime() - time_start;
        }
    }

    var parameters = JSON.parse(document.getElementById('datachannel-parameters').value);
    console.log('DataChannel parameters:', parameters); // Log for debugging

    // Create DataChannel and setup event handlers
    dc = pc.createDataChannel('chat', parameters);
    dc.onclose = function() {
    clearInterval(dcInterval);
    dataChannelLog.textContent += '- close\n';
    console.log('DataChannel closed'); // Log for debugging

    };
    dc.onopen = function() {
        dataChannelLog.textContent += '- open\n';
        console.log('DataChannel opened'); // Log for debugging
        dcInterval = setInterval(function() {
            var message = 'ping ' + current_stamp();
            dataChannelLog.textContent += '> ' + message + '\n';
            dc.send(message);
            console.log('Sent message:', message); // Log for debugging
        }, 1000);
    };
    dc.onmessage = function(evt) {
        dataChannelLog.textContent += '< ' + evt.data + '\n';
        console.log('Received message:', evt.data); // Log for debugging

        if (evt.data.substring(0, 4) === 'pong') {
            var elapsed_ms = current_stamp() - parseInt(evt.data.substring(5), 10);
            dataChannelLog.textContent += ' RTT ' + elapsed_ms + ' ms\n';
            dc.send('latency ' + elapsed_ms);
            console.log('Latency calculated:', elapsed_ms + ' ms'); // Log for debugging
        }
    };

    return negotiate();


}
// Function to stop the peer connection and data channel
function stop() {
    document.getElementById('stop').style.display = 'none';

    if (dc) {
        dc.close();
        console.log('DataChannel closed'); // Log for debugging
    }

    if (pc.getTransceivers) {
        pc.getTransceivers().forEach(function(transceiver) {
            if (transceiver.stop) {
                transceiver.stop();
                console.log('Transceiver stopped:', transceiver); // Log for debugging

            }
        });
    }

    pc.getSenders().forEach(function(sender) {
        sender.track.stop();
        console.log('Sender track stopped:', sender); // Log for debugging

    });

    setTimeout(function() {
        pc.close();
        console.log('Peer connection closed'); // Log for debugging
    }, 500);
}

// Function to filter the SDP for a specific codec
function sdpFilterCodec(kind, codec, realSdp) {
    var allowed = []
    var rtxRegex = new RegExp('a=fmtp:(\\d+) apt=(\\d+)\r$');
    var codecRegex = new RegExp('a=rtpmap:([0-9]+) ' + escapeRegExp(codec))
    var videoRegex = new RegExp('(m=' + kind + ' .*?)( ([0-9]+))*\\s*$')
    
    var lines = realSdp.split('\n');

    var isKind = false;
    for (var i = 0; i < lines.length; i++) {
        if (lines[i].startsWith('m=' + kind + ' ')) {
            isKind = true;
        } else if (lines[i].startsWith('m=')) {
            isKind = false;
        }

        if (isKind) {
            var match = lines[i].match(codecRegex);
            if (match) {
                allowed.push(parseInt(match[1]));
            }

            match = lines[i].match(rtxRegex);
            if (match && allowed.includes(parseInt(match[2]))) {
                allowed.push(parseInt(match[1]));
            }
        }
    }

    var skipRegex = 'a=(fmtp|rtcp-fb|rtpmap):([0-9]+)';
    var sdp = '';

    isKind = false;
    for (var i = 0; i < lines.length; i++) {
        if (lines[i].startsWith('m=' + kind + ' ')) {
            isKind = true;
        } else if (lines[i].startsWith('m=')) {
            isKind = false;
        }

        if (isKind) {
            var skipMatch = lines[i].match(skipRegex);
            if (skipMatch && !allowed.includes(parseInt(skipMatch[2]))) {
                continue;
            } else if (lines[i].match(videoRegex)) {
                sdp += lines[i].replace(videoRegex, '$1 ' + allowed.join(' ')) + '\n';
            } else {
                sdp += lines[i] + '\n';
            }
        } else {
            sdp += lines[i] + '\n';
        }
    }

    console.log('Filtered SDP:', sdp); // Log for debugging
    return sdp;
}

// Function to escape special characters in a string for RegExp
function escapeRegExp(string) {
    return string.replace(/[.*+?^${}()|[\]\\]/g, '\\$&'); 
}
