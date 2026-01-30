---
title: Lab 1
description: Artemis & Bluetooth
pubDate: 2026-02-04
---
<article class="article">

This is a comprehensive Markdown syntax demonstration document showcasing commonly used formatting features.

## Heading

### Sub Heading

This is plain text

**bold text**

*Italicized text*

***text in bold and italics***

~~strikethrough text.~~

<!-- `print('hello')` -->

### Quotes!

> this is reference block
>
> can have multiple lines
>
> > can be nested

## Lists

### Unordered list

- Project 1
- Project 2
    - subproject 2.1
    - subproject 2.2
        - subproject 2.2.1
- Prokect 3

### Ordered List

1. first item
2. second item
    1. sub item 2.1
    2. subitem 2.2
3. third item

### Task List

- [x] completed Tasts
- [ ] TBD
- [ ] another TBD

#### JavaScript example

```javascript
function greet(name) {
    return `Hello, ${name}!`;
}

const message = greet('World');
console.log(message);
```

#### Python exmaple

```python
def fibonacci(n):
    if n <= 1:
        return n
    return fibonacci(n-1) + fibonacci(n-2)

# Generate the Fibonacci sequence
for i in range(10):
    print(f"F({i}) = {fibonacci(i)}")
```

#### CSS example

```css
.example {
    display: flex;
    justify-content: center;
    align-items: center;
    background: linear-gradient(45deg, #ff6b6b, #4ecdc4);
    border-radius: 8px;
    padding: 20px;
    box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
}
```

## links and images

### link

[GitHub](https://github.com)


### picture

![picture](../../../public/pwa.webp)

## table

| function     | describe      | supported? |
|--------|---------|------|
| **title** | multilevel heading supp  | ✅    |
| **list** | ordered/unord lists | ✅    |
| **code** | syntax highlighting    | ✅    |
| **sheet** | data display    | ✅    |
| **linl** | internal/ext links    | ✅    |

## horizontal dividing line 

---
