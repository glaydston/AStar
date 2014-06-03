using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Collections;

namespace ATPS
{
    class Caminho<Node> : IEnumerable<Caminho<Node>>
    {
        public Node LastStep { get; private set; }
        
        public Caminho<Node> PreviousSteps { get; private set; }
        
        public double TotalCost { get; private set; }
        
        private Caminho(Node lastStep, Caminho<Node> previousSteps, double totalCost)
        {
            LastStep = lastStep;
            
            PreviousSteps = previousSteps;
            
            TotalCost = totalCost;
        }

        public Caminho(Node start) : this(start, null, 0) { }
        
        public Caminho<Node> AddStep(Node step, double stepCost)
        {
            return new Caminho<Node>(step, this, TotalCost + stepCost);
        }
        
        public IEnumerator<Caminho<Node>> GetEnumerator()
        {
            for(Caminho<Node> p = this; p != null; p = p.PreviousSteps)
                yield return p;
        }
        
        IEnumerator IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }
    }
}
