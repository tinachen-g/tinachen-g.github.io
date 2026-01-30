import { visit } from 'unist-util-visit';

const RehypeImage = () => {
    return (tree: any) => {
        visit(tree, 'element', (node: any) => {
            if (node.tagName === 'img') {
                node.properties = node.properties || {};
                node.properties.loading = 'lazy';
            }
        });
    };
};

export default RehypeImage;
