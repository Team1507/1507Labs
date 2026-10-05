//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.auto.policy;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * A small JSON reader for the policy file. WPILib 2027 no longer puts Jackson
 * (used by the 2026 ShooterModel) on the robot's classpath, so this avoids a
 * new dependency. The methods read like Jackson's JsonNode:
 * {@code root.get("network").get("layers").get(0).get("inputs").asInt()}.
 *
 * <p>A missing key or index throws with the path, so a malformed policy file
 * fails at load with a clear message instead of driving wrong.
 */
public final class Json implements Iterable<Json> {

    private final Object value;   // Map<String, Json>, List<Json>, Double, String, Boolean, or null
    private final String path;

    private Json(Object value, String path) {
        this.value = value;
        this.path = path;
    }

    /** Reads and parses a JSON file. */
    public static Json read(Path file) throws IOException {
        return parse(Files.readString(file));
    }

    /** Parses JSON text. */
    public static Json parse(String text) {
        Parser parser = new Parser(text);
        Json root = parser.value("$");
        parser.skipSpace();
        if (parser.pos != text.length()) {
            throw parser.error("extra text after the JSON value");
        }
        return root;
    }

    /** The member {@code key} of an object. */
    @SuppressWarnings("unchecked")
    public Json get(String key) {
        Json member = has(key) ? ((Map<String, Json>) value).get(key) : null;
        if (member == null) {
            throw new IllegalArgumentException("policy file: missing " + path + "." + key);
        }
        return member;
    }

    /** True if this is an object with member {@code key}. */
    @SuppressWarnings("unchecked")
    public boolean has(String key) {
        return value instanceof Map && ((Map<String, Json>) value).containsKey(key);
    }

    /** Element {@code index} of an array. */
    public Json get(int index) {
        List<Json> list = list();
        if (index < 0 || index >= list.size()) {
            throw new IllegalArgumentException("policy file: " + path + "[" + index + "] is out of range");
        }
        return list.get(index);
    }

    /** Number of elements of an array. */
    public int size() {
        return list().size();
    }

    public double asDouble() {
        if (!(value instanceof Double)) {
            throw new IllegalArgumentException("policy file: " + path + " is not a number");
        }
        return (Double) value;
    }

    public int asInt() {
        return (int) Math.round(asDouble());
    }

    public String asText() {
        if (!(value instanceof String)) {
            throw new IllegalArgumentException("policy file: " + path + " is not text");
        }
        return (String) value;
    }

    /** An array of numbers as a double[]. */
    public double[] asDoubles() {
        double[] out = new double[size()];
        for (int i = 0; i < out.length; i++) {
            out[i] = get(i).asDouble();
        }
        return out;
    }

    @Override
    public Iterator<Json> iterator() {
        return list().iterator();
    }

    @SuppressWarnings("unchecked")
    private List<Json> list() {
        if (!(value instanceof List)) {
            throw new IllegalArgumentException("policy file: " + path + " is not an array");
        }
        return (List<Json>) value;
    }

    private static final class Parser {
        private final String s;
        private int pos;

        Parser(String s) {
            this.s = s;
        }

        Json value(String path) {
            skipSpace();
            if (pos >= s.length()) {
                throw error("unexpected end");
            }
            char c = s.charAt(pos);
            if (c == '{') {
                pos++;
                Map<String, Json> map = new LinkedHashMap<>();
                skipSpace();
                if (peek() == '}') {
                    pos++;
                    return new Json(map, path);
                }
                while (true) {
                    skipSpace();
                    String key = string();
                    skipSpace();
                    expect(':');
                    map.put(key, value(path + "." + key));
                    skipSpace();
                    if (peek() == ',') {
                        pos++;
                    } else {
                        expect('}');
                        return new Json(map, path);
                    }
                }
            }
            if (c == '[') {
                pos++;
                List<Json> list = new ArrayList<>();
                skipSpace();
                if (peek() == ']') {
                    pos++;
                    return new Json(list, path);
                }
                while (true) {
                    list.add(value(path + "[" + list.size() + "]"));
                    skipSpace();
                    if (peek() == ',') {
                        pos++;
                    } else {
                        expect(']');
                        return new Json(list, path);
                    }
                }
            }
            if (c == '"') {
                return new Json(string(), path);
            }
            if (s.startsWith("true", pos)) {
                pos += 4;
                return new Json(Boolean.TRUE, path);
            }
            if (s.startsWith("false", pos)) {
                pos += 5;
                return new Json(Boolean.FALSE, path);
            }
            if (s.startsWith("null", pos)) {
                pos += 4;
                return new Json(null, path);
            }
            int start = pos;
            while (pos < s.length() && "+-0123456789.eE".indexOf(s.charAt(pos)) >= 0) {
                pos++;
            }
            if (start == pos) {
                throw error("unexpected '" + c + "'");
            }
            return new Json(Double.parseDouble(s.substring(start, pos)), path);
        }

        String string() {
            expect('"');
            StringBuilder out = new StringBuilder();
            while (true) {
                if (pos >= s.length()) {
                    throw error("unterminated string");
                }
                char c = s.charAt(pos++);
                if (c == '"') {
                    return out.toString();
                }
                if (c != '\\') {
                    out.append(c);
                    continue;
                }
                char e = s.charAt(pos++);
                switch (e) {
                    case 'n' -> out.append('\n');
                    case 't' -> out.append('\t');
                    case 'r' -> out.append('\r');
                    case 'b' -> out.append('\b');
                    case 'f' -> out.append('\f');
                    case 'u' -> {
                        out.append((char) Integer.parseInt(s.substring(pos, pos + 4), 16));
                        pos += 4;
                    }
                    default -> out.append(e);   // \" \\ \/
                }
            }
        }

        void skipSpace() {
            while (pos < s.length() && Character.isWhitespace(s.charAt(pos))) {
                pos++;
            }
        }

        char peek() {
            return pos < s.length() ? s.charAt(pos) : '\0';
        }

        void expect(char c) {
            if (peek() != c) {
                throw error("expected '" + c + "'");
            }
            pos++;
        }

        IllegalArgumentException error(String message) {
            return new IllegalArgumentException("policy file: bad JSON at character " + pos + ": " + message);
        }
    }
}
