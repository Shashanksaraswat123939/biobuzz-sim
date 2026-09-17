package sim.bridge;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.WebSocket;
import java.util.Map;
import java.util.concurrent.CompletionStage;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;

/**
 * The only thing between the world and the brain. JDK 11's HttpClient has a WebSocket
 * client built in, so this needs no dependency at all.
 */
public class BridgeClient implements WebSocket.Listener {
    private final LinkedBlockingQueue<SensorFrame> inbox = new LinkedBlockingQueue<SensorFrame>(4);
    private final StringBuilder partial = new StringBuilder();
    private volatile WebSocket ws;
    private volatile boolean closed = false;

    public void connect(String url) throws Exception {
        ws = HttpClient.newHttpClient().newWebSocketBuilder()
                .connectTimeout(java.time.Duration.ofSeconds(10))
                .buildAsync(URI.create(url), this)
                .join();
        send("{\"type\":\"hello\",\"role\":\"brain\"}");
    }

    public boolean isClosed() { return closed; }

    public void send(String text) {
        WebSocket w = ws;
        if (w != null && !closed) w.sendText(text, true);
    }

    /** Blocks for the next SensorFrame, or null if the world went away. */
    public SensorFrame take(long timeoutMs) {
        try {
            return inbox.poll(timeoutMs, TimeUnit.MILLISECONDS);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            return null;
        }
    }

    public void close() {
        closed = true;
        WebSocket w = ws;
        if (w != null) w.abort();
    }

    @Override
    public void onOpen(WebSocket webSocket) { webSocket.request(1); }

    @Override
    public CompletionStage<?> onText(WebSocket webSocket, CharSequence data, boolean last) {
        partial.append(data);
        if (last) {
            String text = partial.toString();
            partial.setLength(0);
            try {
                Map<String, Object> m = Json.parseObject(text);
                if ("sensor".equals(Json.str(m, "type", ""))) {
                    // Drop the oldest rather than block the socket thread: a brain that
                    // falls behind should see the newest world, not a backlog.
                    if (!inbox.offer(new SensorFrame(m))) {
                        inbox.poll();
                        inbox.offer(new SensorFrame(m));
                    }
                }
            } catch (RuntimeException ignored) {
                // a malformed frame is logged by the world, not fatal here
            }
        }
        webSocket.request(1);
        return null;
    }

    @Override
    public CompletionStage<?> onClose(WebSocket webSocket, int statusCode, String reason) {
        closed = true;
        return null;
    }

    @Override
    public void onError(WebSocket webSocket, Throwable error) { closed = true; }
}
