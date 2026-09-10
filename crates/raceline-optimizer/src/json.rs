#[derive(Clone, Debug, PartialEq)]
pub enum JsonValue {
    Null,
    Bool(bool),
    Integer(i64),
    Number(f64),
    String(String),
    Array(Vec<JsonValue>),
    Object(Vec<(String, JsonValue)>),
}

impl JsonValue {
    #[must_use]
    pub fn get(&self, key: &str) -> Option<&JsonValue> {
        match self {
            Self::Object(entries) => entries
                .iter()
                .find(|(entry_key, _)| entry_key == key)
                .map(|(_, value)| value),
            _ => None,
        }
    }

    #[must_use]
    pub fn as_str(&self) -> Option<&str> {
        match self {
            Self::String(value) => Some(value),
            _ => None,
        }
    }

    #[must_use]
    pub fn as_f64(&self) -> Option<f64> {
        match self {
            Self::Integer(value) => Some(*value as f64),
            Self::Number(value) => Some(*value),
            _ => None,
        }
    }

    #[must_use]
    pub fn as_u32(&self) -> Option<u32> {
        self.as_f64().and_then(|value| {
            if value.is_finite()
                && value >= 0.0
                && value.fract() == 0.0
                && value <= f64::from(u32::MAX)
            {
                Some(value as u32)
            } else {
                None
            }
        })
    }

    #[must_use]
    pub fn as_array(&self) -> Option<&[JsonValue]> {
        match self {
            Self::Array(values) => Some(values),
            _ => None,
        }
    }

    #[must_use]
    pub fn to_pretty_string(&self) -> String {
        let mut output = String::new();
        self.write_pretty(&mut output, 0);
        output
    }

    fn write_pretty(&self, output: &mut String, indent: usize) {
        match self {
            Self::Null => output.push_str("null"),
            Self::Bool(value) => output.push_str(if *value { "true" } else { "false" }),
            Self::Integer(value) => output.push_str(&value.to_string()),
            Self::Number(value) => output.push_str(&format_number(*value)),
            Self::String(value) => write_json_string(output, value),
            Self::Array(values) => {
                if values.is_empty() {
                    output.push_str("[]");
                    return;
                }
                output.push('[');
                output.push('\n');
                for (index, value) in values.iter().enumerate() {
                    write_indent(output, indent + 2);
                    value.write_pretty(output, indent + 2);
                    if index + 1 != values.len() {
                        output.push(',');
                    }
                    output.push('\n');
                }
                write_indent(output, indent);
                output.push(']');
            }
            Self::Object(entries) => {
                if entries.is_empty() {
                    output.push_str("{}");
                    return;
                }
                output.push('{');
                output.push('\n');
                for (index, (key, value)) in entries.iter().enumerate() {
                    write_indent(output, indent + 2);
                    write_json_string(output, key);
                    output.push_str(": ");
                    value.write_pretty(output, indent + 2);
                    if index + 1 != entries.len() {
                        output.push(',');
                    }
                    output.push('\n');
                }
                write_indent(output, indent);
                output.push('}');
            }
        }
    }
}

impl From<&str> for JsonValue {
    fn from(value: &str) -> Self {
        Self::String(value.to_owned())
    }
}

impl From<String> for JsonValue {
    fn from(value: String) -> Self {
        Self::String(value)
    }
}

impl From<f64> for JsonValue {
    fn from(value: f64) -> Self {
        Self::Number(value)
    }
}

impl From<u32> for JsonValue {
    fn from(value: u32) -> Self {
        Self::Integer(i64::from(value))
    }
}

impl From<bool> for JsonValue {
    fn from(value: bool) -> Self {
        Self::Bool(value)
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct JsonError {
    pub offset: usize,
    pub message: String,
}

impl std::fmt::Display for JsonError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(formatter, "{} at byte {}", self.message, self.offset)
    }
}

impl std::error::Error for JsonError {}

pub fn parse_json_str(input: &str) -> Result<JsonValue, JsonError> {
    let mut parser = Parser {
        bytes: input.as_bytes(),
        offset: 0,
    };
    let value = parser.parse_value()?;
    parser.skip_ws();
    if parser.offset != parser.bytes.len() {
        return Err(parser.error("trailing data"));
    }
    Ok(value)
}

struct Parser<'a> {
    bytes: &'a [u8],
    offset: usize,
}

impl Parser<'_> {
    fn parse_value(&mut self) -> Result<JsonValue, JsonError> {
        self.skip_ws();
        match self.peek() {
            Some(b'n') => self.parse_literal(b"null", JsonValue::Null),
            Some(b't') => self.parse_literal(b"true", JsonValue::Bool(true)),
            Some(b'f') => self.parse_literal(b"false", JsonValue::Bool(false)),
            Some(b'"') => self.parse_string().map(JsonValue::String),
            Some(b'[') => self.parse_array(),
            Some(b'{') => self.parse_object(),
            Some(b'-' | b'0'..=b'9') => self.parse_number(),
            Some(_) => Err(self.error("unexpected character")),
            None => Err(self.error("unexpected end of input")),
        }
    }

    fn parse_literal(&mut self, expected: &[u8], value: JsonValue) -> Result<JsonValue, JsonError> {
        if self.bytes.get(self.offset..self.offset + expected.len()) == Some(expected) {
            self.offset += expected.len();
            Ok(value)
        } else {
            Err(self.error("invalid literal"))
        }
    }

    fn parse_string(&mut self) -> Result<String, JsonError> {
        self.expect(b'"')?;
        let mut output = String::new();
        while let Some(byte) = self.next() {
            match byte {
                b'"' => return Ok(output),
                b'\\' => output.push(self.parse_escape()?),
                0x00..=0x1f => return Err(self.error("control character in string")),
                _ => {
                    let start = self.offset - 1;
                    let mut end = self.offset;
                    while end < self.bytes.len()
                        && self.bytes[end] != b'"'
                        && self.bytes[end] != b'\\'
                        && self.bytes[end] >= 0x20
                    {
                        end += 1;
                    }
                    let chunk = std::str::from_utf8(&self.bytes[start..end])
                        .map_err(|_| self.error("invalid utf-8 in string"))?;
                    output.push_str(chunk);
                    self.offset = end;
                }
            }
        }
        Err(self.error("unterminated string"))
    }

    fn parse_escape(&mut self) -> Result<char, JsonError> {
        match self.next() {
            Some(b'"') => Ok('"'),
            Some(b'\\') => Ok('\\'),
            Some(b'/') => Ok('/'),
            Some(b'b') => Ok('\u{0008}'),
            Some(b'f') => Ok('\u{000c}'),
            Some(b'n') => Ok('\n'),
            Some(b'r') => Ok('\r'),
            Some(b't') => Ok('\t'),
            Some(b'u') => {
                let code = self.parse_hex4()?;
                char::from_u32(code).ok_or_else(|| self.error("invalid unicode escape"))
            }
            Some(_) => Err(self.error("invalid escape")),
            None => Err(self.error("unterminated escape")),
        }
    }

    fn parse_hex4(&mut self) -> Result<u32, JsonError> {
        let mut value = 0_u32;
        for _ in 0..4 {
            let digit = match self.next() {
                Some(byte @ b'0'..=b'9') => u32::from(byte - b'0'),
                Some(byte @ b'a'..=b'f') => 10 + u32::from(byte - b'a'),
                Some(byte @ b'A'..=b'F') => 10 + u32::from(byte - b'A'),
                _ => return Err(self.error("invalid unicode escape")),
            };
            value = value * 16 + digit;
        }
        Ok(value)
    }

    fn parse_array(&mut self) -> Result<JsonValue, JsonError> {
        self.expect(b'[')?;
        let mut values = Vec::new();
        self.skip_ws();
        if self.consume(b']') {
            return Ok(JsonValue::Array(values));
        }
        loop {
            values.push(self.parse_value()?);
            self.skip_ws();
            if self.consume(b']') {
                break;
            }
            self.expect(b',')?;
        }
        Ok(JsonValue::Array(values))
    }

    fn parse_object(&mut self) -> Result<JsonValue, JsonError> {
        self.expect(b'{')?;
        let mut entries = Vec::new();
        self.skip_ws();
        if self.consume(b'}') {
            return Ok(JsonValue::Object(entries));
        }
        loop {
            self.skip_ws();
            let key = self.parse_string()?;
            self.skip_ws();
            self.expect(b':')?;
            let value = self.parse_value()?;
            entries.push((key, value));
            self.skip_ws();
            if self.consume(b'}') {
                break;
            }
            self.expect(b',')?;
        }
        Ok(JsonValue::Object(entries))
    }

    fn parse_number(&mut self) -> Result<JsonValue, JsonError> {
        let start = self.offset;
        self.consume(b'-');
        self.consume_digits();
        if self.consume(b'.') {
            self.consume_digits();
        }
        if matches!(self.peek(), Some(b'e' | b'E')) {
            self.offset += 1;
            if matches!(self.peek(), Some(b'+' | b'-')) {
                self.offset += 1;
            }
            self.consume_digits();
        }
        let text = std::str::from_utf8(&self.bytes[start..self.offset])
            .map_err(|_| self.error("invalid number"))?;
        let value = text
            .parse::<f64>()
            .map_err(|_| self.error("invalid number"))?;
        if !text.contains('.') && !text.contains('e') && !text.contains('E') {
            if let Ok(value) = text.parse::<i64>() {
                return Ok(JsonValue::Integer(value));
            }
        }
        Ok(JsonValue::Number(value))
    }

    fn consume_digits(&mut self) {
        while matches!(self.peek(), Some(b'0'..=b'9')) {
            self.offset += 1;
        }
    }

    fn skip_ws(&mut self) {
        while matches!(self.peek(), Some(b' ' | b'\n' | b'\r' | b'\t')) {
            self.offset += 1;
        }
    }

    fn expect(&mut self, byte: u8) -> Result<(), JsonError> {
        if self.consume(byte) {
            Ok(())
        } else {
            Err(self.error("unexpected character"))
        }
    }

    fn consume(&mut self, byte: u8) -> bool {
        if self.peek() == Some(byte) {
            self.offset += 1;
            true
        } else {
            false
        }
    }

    fn next(&mut self) -> Option<u8> {
        let byte = self.peek()?;
        self.offset += 1;
        Some(byte)
    }

    fn peek(&self) -> Option<u8> {
        self.bytes.get(self.offset).copied()
    }

    fn error(&self, message: &str) -> JsonError {
        JsonError {
            offset: self.offset,
            message: message.to_owned(),
        }
    }
}

fn write_indent(output: &mut String, indent: usize) {
    for _ in 0..indent {
        output.push(' ');
    }
}

fn write_json_string(output: &mut String, value: &str) {
    output.push('"');
    for ch in value.chars() {
        match ch {
            '"' => output.push_str("\\\""),
            '\\' => output.push_str("\\\\"),
            '\n' => output.push_str("\\n"),
            '\r' => output.push_str("\\r"),
            '\t' => output.push_str("\\t"),
            '\u{0008}' => output.push_str("\\b"),
            '\u{000c}' => output.push_str("\\f"),
            ch if ch < '\u{0020}' => output.push_str(&format!("\\u{:04x}", ch as u32)),
            ch => output.push(ch),
        }
    }
    output.push('"');
}

fn format_number(value: f64) -> String {
    if value.is_finite() && value.fract() == 0.0 {
        format!("{value:.1}")
    } else {
        format!("{value}")
    }
}

#[cfg(test)]
mod tests {
    use super::{parse_json_str, JsonValue};

    #[test]
    fn parses_nested_json_and_preserves_object_order() {
        let value = parse_json_str(r#"{"a": 1, "b": [true, null, "x"]}"#).unwrap();

        assert_eq!(
            value,
            JsonValue::Object(vec![
                ("a".to_owned(), JsonValue::Integer(1)),
                (
                    "b".to_owned(),
                    JsonValue::Array(vec![
                        JsonValue::Bool(true),
                        JsonValue::Null,
                        JsonValue::String("x".to_owned())
                    ])
                )
            ])
        );
    }

    #[test]
    fn writes_pretty_json() {
        let value = JsonValue::Object(vec![(
            "points".to_owned(),
            JsonValue::Array(vec![JsonValue::Array(vec![1.0.into(), 2.5.into()])]),
        )]);

        assert_eq!(
            value.to_pretty_string(),
            "{\n  \"points\": [\n    [\n      1.0,\n      2.5\n    ]\n  ]\n}"
        );
    }
}
