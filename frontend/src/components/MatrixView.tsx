export function MatrixView({ matrix, digits = 3 }: { matrix: number[][]; digits?: number }) {
  return (
    <div className="matrix" role="table">
      {matrix.map((row, rowIndex) => (
        <div className="matrix-row" role="row" key={rowIndex}>
          {row.map((value, columnIndex) => (
            <span role="cell" key={columnIndex}>{Number.isFinite(value) ? value.toFixed(digits) : "—"}</span>
          ))}
        </div>
      ))}
    </div>
  );
}

