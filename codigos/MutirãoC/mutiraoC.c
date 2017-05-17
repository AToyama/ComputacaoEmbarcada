#include <stdio.h>

void main(void) {
    
    //declarações
    char produto1[25], produto2[25], produto3[25];
    float p1, p2, p3;
    int qtd1, qtd2, qtd3;
    
    //entrada do item 1
    printf("insira o primeiro item (nome, preço e quantidade): ");
    scanf("%s %f %d",produto1, &p1, &qtd1);

    //entrada item 2
    printf("insira o segundo item (nome, preço e quantidade): ");
    scanf("%s %f %d",produto2, &p2, &qtd2);
    
    //entrada item 3
    printf("insira o terceiro item (nome, preço e quantidade): ");
    scanf("%s %f %d",produto3, &p3, &qtd3);
    
    //imprime a tabela
    printf("\nProduto | Preço Unitário | Quantidade | Subtotal\n");
    printf("%s   |      %.2f      |       %d      |    %.2f\n",produto1,p1,qtd1,p1*qtd1);
    printf("%s   |      %.2f      |       %d      |    %.2f\n",produto2,p2,qtd2,p2*qtd2);
    printf("%s   |      %.2f      |       %d      |    %.2f\n",produto3,p3,qtd3,p3*qtd3);
    printf("Preço total: R$%.2f\n", p1*qtd1+p2*qtd2+p3*qtd3);
}
