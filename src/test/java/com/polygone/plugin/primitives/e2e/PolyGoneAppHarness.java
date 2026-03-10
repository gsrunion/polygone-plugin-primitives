package com.polygone.plugin.primitives.e2e;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.IOException;
import java.net.ServerSocket;
import java.net.URI;
import java.net.URLEncoder;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.fail;

final class PolyGoneAppHarness implements AutoCloseable {

    private static final Duration STARTUP_TIMEOUT = Duration.ofSeconds(45);
    private static final Duration REQUEST_TIMEOUT = Duration.ofSeconds(60);

    private final HttpClient httpClient;
    private final ObjectMapper objectMapper;
    private final Process process;
    private final Path logFile;
    private final int port;

    private PolyGoneAppHarness(Process process, Path logFile, int port) {
        this.httpClient = HttpClient.newBuilder()
                .connectTimeout(Duration.ofSeconds(2))
                .build();
        this.objectMapper = new ObjectMapper();
        this.process = process;
        this.logFile = logFile;
        this.port = port;
    }

    static PolyGoneAppHarness launch(Path projectRoot, Path userHome, int port) throws Exception {
        Files.createDirectories(userHome);
        Path logsDir = userHome.resolve("logs");
        Files.createDirectories(logsDir);
        Path logFile = logsDir.resolve("polygone-e2e.log");

        String javaBin = Path.of(System.getProperty("java.home"), "bin", "java").toString();
        String classpath = System.getProperty("java.class.path");

        List<String> command = new ArrayList<>();
        command.add(javaBin);
        command.add("-cp");
        command.add(classpath);
        command.add("-Duser.home=" + userHome.toAbsolutePath());
        command.add("com.polygone.app.PolyGoneApplication");
        command.add("--polygone.test-hook.enabled=true");
        command.add("--polygone.test-hook.port=" + port);
        command.add("--polygone.plugins.directory=" + userHome.resolve(".polygone/plugins").toAbsolutePath());
        command.add("--polygone.plugins.builtin-directory=nonexistent");

        ProcessBuilder processBuilder = new ProcessBuilder(command);
        processBuilder.directory(projectRoot.toFile());
        processBuilder.redirectErrorStream(true);
        processBuilder.redirectOutput(logFile.toFile());

        PolyGoneAppHarness harness = new PolyGoneAppHarness(processBuilder.start(), logFile, port);
        harness.waitForStartup();
        return harness;
    }

    static int findFreePort() throws IOException {
        try (ServerSocket socket = new ServerSocket(0)) {
            return socket.getLocalPort();
        }
    }

    static boolean canRunUiProcess() {
        String display = System.getenv("DISPLAY");
        return display != null && !display.isBlank();
    }

    Map<String, Object> get(String endpoint, Map<String, String> params) throws Exception {
        return send("GET", endpoint, params);
    }

    Map<String, Object> post(String endpoint, Map<String, String> params) throws Exception {
        return send("POST", endpoint, params);
    }

    String readLogs() throws IOException {
        return Files.exists(logFile) ? Files.readString(logFile) : "";
    }

    @Override
    public void close() throws Exception {
        if (!process.isAlive()) {
            return;
        }
        process.destroy();
        if (!process.waitFor(10, java.util.concurrent.TimeUnit.SECONDS)) {
            process.destroyForcibly();
            process.waitFor(10, java.util.concurrent.TimeUnit.SECONDS);
        }
    }

    private void waitForStartup() throws Exception {
        Instant deadline = Instant.now().plus(STARTUP_TIMEOUT);
        Exception lastFailure = null;
        while (Instant.now().isBefore(deadline)) {
            if (!process.isAlive()) {
                fail("PolyGone process exited during startup.\n" + readLogs());
            }
            try {
                Map<String, Object> status = get("/api/status", Map.of());
                if (status.containsKey("meshLoaded")) {
                    return;
                }
            } catch (Exception e) {
                lastFailure = e;
            }
            Thread.sleep(250);
        }
        StringBuilder message = new StringBuilder("Timed out waiting for PolyGone HTTP hook");
        if (lastFailure != null) {
            message.append(": ").append(lastFailure.getMessage());
        }
        message.append('\n').append(readLogs());
        fail(message.toString());
    }

    private Map<String, Object> send(String method, String endpoint, Map<String, String> params) throws Exception {
        if (!process.isAlive()) {
            fail("PolyGone process is not running.\n" + readLogs());
        }
        HttpRequest.Builder builder = HttpRequest.newBuilder()
                .uri(buildUri(endpoint, params))
                .timeout(REQUEST_TIMEOUT);
        if ("POST".equals(method)) {
            builder.POST(HttpRequest.BodyPublishers.noBody());
        } else {
            builder.GET();
        }
        HttpResponse<String> response = httpClient.send(builder.build(), HttpResponse.BodyHandlers.ofString());
        if (response.statusCode() >= 400) {
            fail("HTTP " + response.statusCode() + " for " + endpoint + ": " + response.body() + "\n" + readLogs());
        }
        return objectMapper.readValue(response.body(), new TypeReference<>() {});
    }

    private URI buildUri(String endpoint, Map<String, String> params) {
        StringBuilder uri = new StringBuilder("http://127.0.0.1:")
                .append(port)
                .append(endpoint);
        if (!params.isEmpty()) {
            uri.append('?');
            boolean first = true;
            for (Map.Entry<String, String> entry : params.entrySet()) {
                if (!first) {
                    uri.append('&');
                }
                first = false;
                uri.append(URLEncoder.encode(entry.getKey(), StandardCharsets.UTF_8));
                uri.append('=');
                uri.append(URLEncoder.encode(entry.getValue(), StandardCharsets.UTF_8));
            }
        }
        return URI.create(uri.toString());
    }
}
